import argparse
import csv
import json
import os
import pprint
import re
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Union

from main_tracking_v9 import (
    DEVICE_THETA,
    GIMBAL_AZ_BASE,
    HARDWARE_MAP,
    IMG_H,
    IMG_W,
    angular_diff,
    calculate_angles,
    get_camera_params,
    ui_to_ctrl_angles,
)

try:
    from main_tracking_v9 import GIMBAL_PORT
except ImportError:
    GIMBAL_PORT = None


FOV_X = 17.5
FOV_Y = 9.9

# ============================================================
# User-editable defaults.
# You can run this script directly after editing this block:
#     python calibration_matrix_to_gimbal.py
# Command-line arguments still override these values.
# ============================================================
DEFAULT_BOARD = "BOARD_2"
DEFAULT_CAM = 4  # 0-based cam index: layer-2 second camera is BOARD_2 / cam 1.
DEFAULT_MODE = "camera"
DEFAULT_LOWER_CAMERA = "000000005"
DEFAULT_GIMBAL_CAMERA = "000000008"
DEFAULT_TARGET_NAME = "target_A"
DEFAULT_RECORD_CSV = "calibration_records_abs_theta.csv"
DEFAULT_GIMBAL_PORT = GIMBAL_PORT or os.getenv("GIMBAL_PORT") or "COM8"
DEFAULT_ENABLE_GIMBAL_MOVE = True
DEFAULT_MANUAL_TRUE_ANGLE = True
DEFAULT_GIMBAL_CAM_FOV_X = 17.5
DEFAULT_GIMBAL_CAM_FOV_Y = 9.9
DEFAULT_OVERLAY_SCALE = 1.5
# Manual mode is enabled by default. Set DEFAULT_MANUAL_TRUE_ANGLE = False
# if you want to use gimbal-camera click/FOV mode instead.

CSV_FIELDS = [
    "timestamp",
    "board",
    "cam",
    "logic_id",
    "target_name",
    "distance_est",
    "mode",
    "lower_image_path",
    "gimbal_image_path",
    "lower_w",
    "lower_h",
    "lower_x",
    "lower_y",
    "lower_dx",
    "lower_dy",
    "lower_offset_az",
    "lower_offset_el",
    "old_theta_horizontal",
    "old_theta_vertical",
    "calc_ui_az",
    "calc_ui_el",
    "ctrl_az",
    "ctrl_el",
    "gimbal_w",
    "gimbal_h",
    "gimbal_x",
    "gimbal_y",
    "gimbal_dx",
    "gimbal_dy",
    "gimbal_cam_fov_x",
    "gimbal_cam_fov_y",
    "gimbal_offset_az",
    "gimbal_offset_el",
    "manual_true_angle",
    "true_ctrl_az",
    "true_ctrl_el",
    "true_ui_az",
    "true_ui_el",
    "delta_az",
    "delta_el",
    "new_theta_horizontal",
    "new_theta_vertical",
    "valid",
    "note",
]


SUMMARY_FIELDS = [
    "logic_id",
    "count",
    "old_theta_horizontal",
    "old_theta_vertical",
    "delta_az_median",
    "delta_az_mean",
    "delta_az_std",
    "delta_az_min",
    "delta_az_max",
    "delta_el_median",
    "delta_el_mean",
    "delta_el_std",
    "delta_el_min",
    "delta_el_max",
    "final_delta_az_median",
    "final_delta_el_median",
    "suggested_theta_horizontal",
    "suggested_theta_vertical",
    "suspicious_az_count",
    "suspicious_el_count",
    "strong_warning_count",
]


class CalibrationError(Exception):
    pass


_WINDOWS_CAMERA_CACHE: Optional[List[Dict[str, str]]] = None


@dataclass
class CalibrationRecord:
    timestamp: str = ""
    board: str = ""
    cam: int = 0
    logic_id: int = 0
    target_name: str = ""
    distance_est: str = ""
    mode: str = ""
    lower_image_path: str = ""
    gimbal_image_path: str = ""
    lower_w: int = 0
    lower_h: int = 0
    lower_x: float = 0.0
    lower_y: float = 0.0
    lower_dx: float = 0.0
    lower_dy: float = 0.0
    lower_offset_az: float = 0.0
    lower_offset_el: float = 0.0
    old_theta_horizontal: float = 0.0
    old_theta_vertical: float = 0.0
    calc_ui_az: float = 0.0
    calc_ui_el: float = 0.0
    ctrl_az: float = 0.0
    ctrl_el: float = 0.0
    gimbal_w: str = ""
    gimbal_h: str = ""
    gimbal_x: str = ""
    gimbal_y: str = ""
    gimbal_dx: str = ""
    gimbal_dy: str = ""
    gimbal_cam_fov_x: str = ""
    gimbal_cam_fov_y: str = ""
    gimbal_offset_az: str = ""
    gimbal_offset_el: str = ""
    manual_true_angle: int = 0
    true_ctrl_az: str = ""
    true_ctrl_el: str = ""
    true_ui_az: str = ""
    true_ui_el: str = ""
    delta_az: float = 0.0
    delta_el: float = 0.0
    new_theta_horizontal: float = 0.0
    new_theta_vertical: float = 0.0
    valid: int = 1
    note: str = ""

    def to_row(self) -> Dict[str, object]:
        row = asdict(self)
        return {field: row.get(field, "") for field in CSV_FIELDS}


def import_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise CalibrationError(
            "OpenCV is required for image/camera calibration. Install opencv-python first."
        ) from exc
    return cv2


def resolve_camera_params(board: str, cam: int) -> Tuple[int, Dict[str, float]]:
    logic_id, cfg = get_camera_params(board, cam)
    if logic_id is not None and cfg is not None:
        return int(logic_id), cfg

    key = (str(board), int(cam))
    if key not in HARDWARE_MAP:
        raise CalibrationError(f"board/cam is not in HARDWARE_MAP: board={board}, cam={cam}")
    logic_id = int(HARDWARE_MAP[key])
    if logic_id not in DEVICE_THETA:
        raise CalibrationError(f"logic_id is not in DEVICE_THETA: logic_id={logic_id}")
    return logic_id, DEVICE_THETA[logic_id]


def compute_camera_angle_from_click(
    click_x: float,
    click_y: float,
    image_w: float,
    image_h: float,
    theta_cfg: Dict[str, float],
    fov_x: float = FOV_X,
    fov_y: float = FOV_Y,
) -> Dict[str, float]:
    if image_w <= 0 or image_h <= 0:
        raise CalibrationError(f"invalid lower image size: {image_w}x{image_h}")

    dx = float(click_x) - image_w / 2.0
    dy = float(click_y) - image_h / 2.0
    offset_az = dx * float(fov_x) / image_w
    offset_el = -dy * float(fov_y) / image_h
    calc_ui_az = (float(theta_cfg["theta_horizontal"]) + offset_az) % 360.0
    calc_ui_el = float(theta_cfg["theta_vertical"]) + offset_el

    # Keep the main-program function imported and cross-check its fixed-size path.
    if abs(image_w - IMG_W) < 1e-6 and abs(image_h - IMG_H) < 1e-6:
        main_az, main_el = calculate_angles(None, click_x, click_y, cfg=theta_cfg)
        if abs(angular_diff(calc_ui_az, main_az)) > 1e-6 or abs(calc_ui_el - main_el) > 1e-6:
            raise CalibrationError("internal angle check disagrees with main_tracking_v9.calculate_angles()")

    return {
        "dx": dx,
        "dy": dy,
        "offset_az": offset_az,
        "offset_el": offset_el,
        "calc_ui_az": calc_ui_az,
        "calc_ui_el": calc_ui_el,
    }


def compute_gimbal_offset_from_click(
    click_x: float,
    click_y: float,
    image_w: float,
    image_h: float,
    gimbal_cam_fov_x: float,
    gimbal_cam_fov_y: float,
) -> Dict[str, float]:
    if image_w <= 0 or image_h <= 0:
        raise CalibrationError(f"invalid gimbal image size: {image_w}x{image_h}")
    if gimbal_cam_fov_x is None or gimbal_cam_fov_y is None:
        raise CalibrationError("gimbal camera FOV is required unless --manual-true-angle is used")

    dx = float(click_x) - image_w / 2.0
    dy = float(click_y) - image_h / 2.0
    offset_az = dx * float(gimbal_cam_fov_x) / image_w
    offset_el = -dy * float(gimbal_cam_fov_y) / image_h
    return {
        "dx": dx,
        "dy": dy,
        "offset_az": offset_az,
        "offset_el": offset_el,
    }


def compute_delta_by_manual_true_angle(
    calc_ui_az: float,
    calc_ui_el: float,
    true_ctrl_az: float,
    true_ctrl_el: float,
) -> Dict[str, float]:
    true_ui_az = float(true_ctrl_az) % 360.0
    true_ui_el = float(true_ctrl_el)
    delta_az = angular_diff(true_ui_az, calc_ui_az)
    delta_el = true_ui_el - float(calc_ui_el)
    return {
        "true_ui_az": true_ui_az,
        "true_ui_el": true_ui_el,
        "delta_az": delta_az,
        "delta_el": delta_el,
    }


def stored_theta_to_calibration_theta(theta_cfg: Dict[str, float]) -> Dict[str, float]:
    return {
        "theta_horizontal": (float(theta_cfg["theta_horizontal"]) + GIMBAL_AZ_BASE) % 360.0,
        "theta_vertical": float(theta_cfg["theta_vertical"]),
    }


def calibration_ui_to_ctrl_angles(ui_az: float, ui_el: float) -> Tuple[float, float]:
    ctrl_az = float(ui_az) % 360.0
    ctrl_el = float(ui_el)
    if ctrl_az < 0.0:
        ctrl_az = 0.0
    if ctrl_az > 350.0:
        ctrl_az = 350.0
    return ctrl_az, ctrl_el


def append_record_csv(path: str, record: CalibrationRecord) -> None:
    need_header = not os.path.exists(path) or os.path.getsize(path) == 0
    if not need_header:
        with open(path, "rb+") as f:
            f.seek(0, os.SEEK_END)
            if f.tell() > 0:
                f.seek(-1, os.SEEK_END)
                if f.read(1) not in (b"\n", b"\r"):
                    f.write(b"\n")
    with open(path, "a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        if need_header:
            writer.writeheader()
        writer.writerow(record.to_row())


def _read_float(row: Dict[str, str], field: str) -> float:
    try:
        return float(row.get(field, ""))
    except (TypeError, ValueError):
        raise CalibrationError(f"invalid numeric value in field {field}: {row.get(field)!r}")


def _is_valid_row(row: Dict[str, str]) -> bool:
    return str(row.get("valid", "")).strip() in ("1", "true", "True", "yes", "YES")


def _stats(values: List[float]) -> Dict[str, float]:
    return {
        "median": statistics.median(values),
        "mean": statistics.mean(values),
        "std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def summarize_records(
    records_csv: str,
    summary_csv: str = "calibration_summary.csv",
    suggested_output: str = "suggested_DEVICE_THETA.py",
) -> List[Dict[str, object]]:
    if not os.path.exists(records_csv):
        raise CalibrationError(f"records CSV not found: {records_csv}")

    with open(records_csv, "r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    groups: Dict[int, List[Dict[str, str]]] = {}
    for row in rows:
        if not _is_valid_row(row):
            continue
        try:
            logic_id = int(float(row.get("logic_id", "")))
        except (TypeError, ValueError):
            continue
        groups.setdefault(logic_id, []).append(row)

    summaries: List[Dict[str, object]] = []
    suggested_theta = {
        int(k): {
            "theta_vertical": float(v["theta_vertical"]),
            "theta_horizontal": (float(v["theta_horizontal"]) + GIMBAL_AZ_BASE) % 360.0,
        }
        for k, v in DEVICE_THETA.items()
    }

    for logic_id in sorted(groups):
        group = groups[logic_id]
        delta_az_values = [_read_float(row, "delta_az") for row in group]
        delta_el_values = [_read_float(row, "delta_el") for row in group]
        az_stats = _stats(delta_az_values)
        el_stats = _stats(delta_el_values)

        old_horizontal = statistics.median([_read_float(row, "old_theta_horizontal") for row in group])
        old_vertical = statistics.median([_read_float(row, "old_theta_vertical") for row in group])

        suspicious_az = sum(abs(v - az_stats["median"]) > 0.5 for v in delta_az_values)
        suspicious_el = sum(abs(v - el_stats["median"]) > 0.5 for v in delta_el_values)
        strong_warning = sum(
            abs(az) > 3.0 or abs(el) > 3.0
            for az, el in zip(delta_az_values, delta_el_values)
        )

        suggested_horizontal = (old_horizontal + az_stats["median"]) % 360.0
        suggested_vertical = old_vertical + el_stats["median"]
        suggested_theta[logic_id] = {
            "theta_vertical": suggested_vertical,
            "theta_horizontal": suggested_horizontal,
        }

        summaries.append({
            "logic_id": logic_id,
            "count": len(group),
            "old_theta_horizontal": old_horizontal,
            "old_theta_vertical": old_vertical,
            "delta_az_median": az_stats["median"],
            "delta_az_mean": az_stats["mean"],
            "delta_az_std": az_stats["std"],
            "delta_az_min": az_stats["min"],
            "delta_az_max": az_stats["max"],
            "delta_el_median": el_stats["median"],
            "delta_el_mean": el_stats["mean"],
            "delta_el_std": el_stats["std"],
            "delta_el_min": el_stats["min"],
            "delta_el_max": el_stats["max"],
            "final_delta_az_median": az_stats["median"],
            "final_delta_el_median": el_stats["median"],
            "suggested_theta_horizontal": suggested_horizontal,
            "suggested_theta_vertical": suggested_vertical,
            "suspicious_az_count": suspicious_az,
            "suspicious_el_count": suspicious_el,
            "strong_warning_count": strong_warning,
        })

    with open(summary_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(summaries)

    with open(suggested_output, "w", encoding="utf-8", newline="\n") as f:
        f.write("# Generated by calibration_matrix_to_gimbal.py.\n")
        f.write("# Values are in calibration space: DEVICE_THETA.theta_horizontal + GIMBAL_AZ_BASE.\n")
        f.write("# Do not copy directly into main_tracking_v9.py until you subtract the chosen baseline camera angle.\n")
        f.write("DEVICE_THETA = ")
        f.write(pprint.pformat(suggested_theta, sort_dicts=True, width=120))
        f.write("\n")

    for summary in summaries:
        print(
            "logic_id={logic_id} count={count} "
            "delta_az_median={final_delta_az_median:.6f} "
            "delta_el_median={final_delta_el_median:.6f} "
            "suggested=({suggested_theta_horizontal:.6f}, {suggested_theta_vertical:.6f}) "
            "suspicious=az:{suspicious_az_count},el:{suspicious_el_count} "
            "strong_warning={strong_warning_count}".format(**summary)
        )
        if int(summary["strong_warning_count"]) > 0:
            print(
                "  WARNING: abs(delta) > 3 deg. Check board/cam mapping, click point, image size/FOV, "
                "gimbal conversion, or initial DEVICE_THETA."
            )

    print(f"Wrote {summary_csv}")
    print(f"Wrote {suggested_output}")
    return summaries


def read_image_frame(path: str):
    cv2 = import_cv2()
    frame = cv2.imread(path, cv2.IMREAD_COLOR)
    if frame is None:
        raise CalibrationError(f"image read failed: {path}")
    return frame


def enumerate_windows_camera_names() -> List[Dict[str, str]]:
    global _WINDOWS_CAMERA_CACHE
    if os.name != "nt":
        return []
    if _WINDOWS_CAMERA_CACHE is not None:
        return list(_WINDOWS_CAMERA_CACHE)
    command = (
        "Get-CimInstance Win32_PnPEntity | "
        "Where-Object { $_.PNPClass -eq 'Camera' -and $_.Status -eq 'OK' } | "
        "Select-Object Name,Status,PNPDeviceID | ConvertTo-Json -Depth 3"
    )
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", command],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=20,
            check=False,
        )
    except Exception as exc:
        raise CalibrationError(f"failed to enumerate Windows cameras: {exc}") from exc
    if result.returncode != 0:
        raise CalibrationError(f"failed to enumerate Windows cameras: {result.stderr.strip()}")
    text = result.stdout.strip()
    if not text:
        return []
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:
        raise CalibrationError(f"failed to parse Windows camera list: {exc}") from exc
    if isinstance(parsed, dict):
        parsed = [parsed]
    cameras = []
    for item in parsed:
        name = str(item.get("Name", "")).strip()
        pnp_id = str(item.get("PNPDeviceID", "")).strip()
        if name:
            cameras.append({"name": name, "pnp_device_id": pnp_id})
    _WINDOWS_CAMERA_CACHE = list(cameras)
    return cameras


def enumerate_dshow_video_devices() -> List[str]:
    if os.name != "nt":
        return []
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=20,
            check=False,
        )
    except FileNotFoundError:
        return []
    except Exception as exc:
        raise CalibrationError(f"failed to enumerate DirectShow cameras with ffmpeg: {exc}") from exc

    text = (result.stderr or "") + "\n" + (result.stdout or "")
    devices = []
    for line in text.splitlines():
        match = re.search(r'\]\s+"(.+)"\s+\(video\)', line)
        if match:
            devices.append(match.group(1))
    return devices


def resolve_camera_source(camera_ref: Union[int, str]) -> Tuple[Union[int, str], str]:
    raw = str(camera_ref).strip()
    if not raw:
        raise CalibrationError("empty camera reference")

    if os.name == "nt":
        dshow_devices = enumerate_dshow_video_devices()
        dshow_matches = [idx for idx, name in enumerate(dshow_devices) if name == raw]
        if dshow_matches:
            if len(dshow_matches) > 1:
                raise CalibrationError(
                    f"multiple DirectShow video devices named {raw!r}; use explicit index. "
                    f"DirectShow devices: {', '.join(f'{i}:{name}' for i, name in enumerate(dshow_devices))}"
                )
            idx = dshow_matches[0]
            print(f"[Camera] DirectShow camera name {raw!r} resolved to OpenCV index {idx}")
            return idx, f"{raw}(dshow-index:{idx})"
        if raw.startswith("0") and len(raw) > 1 and raw.isdigit():
            available = ", ".join(f"{i}:{name}" for i, name in enumerate(dshow_devices)) or "none"
            raise CalibrationError(
                f"DirectShow camera name {raw!r} was not found. "
                f"Current DirectShow video devices: {available}"
            )

        cameras = enumerate_windows_camera_names()
        if any(cam["name"] == raw for cam in cameras):
            available = ", ".join(f"{i}:{name}" for i, name in enumerate(dshow_devices)) or "none"
            raise CalibrationError(
                f"Windows camera name {raw!r} is visible, but this OpenCV build cannot open cameras "
                "by device-manager name directly and it was not found in DirectShow video devices. "
                "Use an explicit OpenCV index such as 1, or a backend-prefixed index such as dshow:2. "
                f"DirectShow devices: {available}"
            )

    try:
        return int(raw), f"index:{int(raw)}"
    except ValueError:
        if os.name == "nt":
            available = ", ".join(cam["name"] for cam in enumerate_windows_camera_names()) or "none"
            raise CalibrationError(
                f"camera {raw!r} is not an integer index. Use an explicit OpenCV index such as 1, "
                f"or a backend-prefixed index such as dshow:1 / msmf:2. "
                f"Visible Windows camera names for reference: {available}"
            )
        raise CalibrationError(f"camera {raw!r} is not an integer index")


def parse_camera_ref(camera_ref: Union[int, str]) -> Tuple[Union[int, str], int, str]:
    cv2 = import_cv2()
    raw = str(camera_ref).strip()
    backend_name = "dshow" if os.name == "nt" else "any"
    source_text = raw
    if ":" in raw:
        prefix, rest = raw.split(":", 1)
        prefix = prefix.strip().lower()
        if prefix in ("dshow", "msmf", "any"):
            backend_name = prefix
            source_text = rest.strip()

    source, label = resolve_camera_source(source_text)
    if backend_name == "dshow":
        backend = cv2.CAP_DSHOW
    elif backend_name == "msmf":
        backend = cv2.CAP_MSMF
    else:
        backend = cv2.CAP_ANY
    return source, backend, f"{backend_name}:{label}"


def open_camera_capture(camera_ref: Union[int, str], width: int = int(IMG_W), height: int = int(IMG_H)):
    cv2 = import_cv2()
    camera_source, backend, camera_label = parse_camera_ref(camera_ref)
    cap = cv2.VideoCapture(camera_source, backend)
    if not cap.isOpened():
        raise CalibrationError(f"camera open failed: {camera_ref} ({camera_label})")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    return cap, camera_label


def read_camera_frame(camera_ref: Union[int, str], width: int = int(IMG_W), height: int = int(IMG_H)):
    cap, camera_label = open_camera_capture(camera_ref, width, height)
    ok = False
    frame = None
    for _ in range(3):
        ok, frame = cap.read()
        if ok and frame is not None:
            break
    cap.release()
    if not ok or frame is None:
        raise CalibrationError(f"camera frame read failed: {camera_ref} ({camera_label})")
    actual_h, actual_w = frame.shape[:2]
    if int(actual_w) != int(width) or int(actual_h) != int(height):
        print(
            f"[Warn] camera {camera_label} requested {int(width)}x{int(height)}, "
            f"actual frame is {actual_w}x{actual_h}"
        )
    return frame


def load_frame_for_phase(args, phase: str):
    if phase == "lower":
        if args.mode == "image":
            if not args.lower_image:
                raise CalibrationError("--lower-image is required in image mode")
            return read_image_frame(args.lower_image), args.lower_image
        if args.lower_camera is None:
            raise CalibrationError("--lower-camera is required in camera mode")
        return read_camera_frame(args.lower_camera, args.lower_camera_w, args.lower_camera_h), f"camera:{args.lower_camera}"

    if args.mode == "image":
        if not args.gimbal_image:
            raise CalibrationError("--gimbal-image is required unless --manual-true-angle is used")
        return read_image_frame(args.gimbal_image), args.gimbal_image
    if args.gimbal_camera is None:
        raise CalibrationError("--gimbal-camera is required unless --manual-true-angle is used")
    return read_camera_frame(args.gimbal_camera, args.gimbal_camera_w, args.gimbal_camera_h), f"camera:{args.gimbal_camera}"


def draw_overlay(
    frame,
    click,
    lines: Iterable[str],
    valid: int,
    window_title: str,
    overlay_scale: float = 1.15,
):
    cv2 = import_cv2()
    canvas = frame.copy()
    h, w = canvas.shape[:2]
    cx = int(round(w / 2.0))
    cy = int(round(h / 2.0))
    line_thickness = max(2, int(round(2 * overlay_scale)))
    text_thickness = max(2, int(round(2 * overlay_scale)))
    shadow_thickness = text_thickness + 3
    font_scale = max(0.4, float(overlay_scale))
    line_step = int(round(42 * font_scale))
    x_margin = int(round(22 * font_scale))
    y = int(round(44 * font_scale))

    cv2.line(canvas, (cx, 0), (cx, h - 1), (0, 255, 255), line_thickness)
    cv2.line(canvas, (0, cy), (w - 1, cy), (0, 255, 255), line_thickness)
    cv2.circle(canvas, (cx, cy), max(5, int(round(6 * overlay_scale))), (0, 255, 255), -1)

    if click is not None:
        px, py = int(round(click[0])), int(round(click[1]))
        cv2.circle(canvas, (px, py), max(7, int(round(8 * overlay_scale))), (0, 0, 255), -1)
        cv2.line(canvas, (cx, cy), (px, py), (0, 0, 255), line_thickness)

    for line in list(lines) + [f"valid={valid}", "left click target | r retry | v toggle valid | s accept | q quit"]:
        cv2.putText(
            canvas,
            str(line),
            (x_margin, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),
            shadow_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            str(line),
            (x_margin, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            text_thickness,
            cv2.LINE_AA,
        )
        y += line_step

    cv2.imshow(window_title, canvas)


def interactive_click(
    frame,
    window_title: str,
    line_builder,
    initial_valid: int = 1,
    overlay_scale: float = 1.15,
) -> Tuple[float, float, int]:
    cv2 = import_cv2()
    state = {"click": None, "valid": int(initial_valid)}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (float(x), float(y))

    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_title, on_mouse)

    try:
        while True:
            lines = line_builder(state["click"])
            draw_overlay(frame, state["click"], lines, state["valid"], window_title, overlay_scale)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("r"):
                state["click"] = None
            elif key == ord("v"):
                state["valid"] = 0 if state["valid"] else 1
            elif key == ord("s"):
                if state["click"] is None:
                    print("No click yet. Left-click the target first.")
                    continue
                return state["click"][0], state["click"][1], state["valid"]
            elif key == ord("q") or key == 27:
                raise CalibrationError(f"user aborted in window: {window_title}")
    finally:
        cv2.destroyWindow(window_title)


def interactive_camera_click(
    camera_ref: Union[int, str],
    width: int,
    height: int,
    window_title: str,
    line_builder,
    initial_valid: int = 1,
    overlay_scale: float = 1.15,
) -> Tuple[object, float, float, int, str]:
    cv2 = import_cv2()
    cap, camera_label = open_camera_capture(camera_ref, width, height)
    state = {"click": None, "valid": int(initial_valid), "frozen_frame": None}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and state["current_frame"] is not None:
            state["click"] = (float(x), float(y))
            state["frozen_frame"] = state["current_frame"].copy()

    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_title, on_mouse)
    state["current_frame"] = None
    warned_size = False

    try:
        while True:
            if state["frozen_frame"] is None:
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise CalibrationError(f"camera frame read failed: {camera_ref} ({camera_label})")
                state["current_frame"] = frame
                actual_h, actual_w = frame.shape[:2]
                if not warned_size and (int(actual_w) != int(width) or int(actual_h) != int(height)):
                    print(
                        f"[Warn] camera {camera_label} requested {int(width)}x{int(height)}, "
                        f"actual frame is {actual_w}x{actual_h}"
                    )
                    warned_size = True
            else:
                frame = state["frozen_frame"]
                state["current_frame"] = frame

            lines = line_builder(frame, state["click"])
            if state["frozen_frame"] is None:
                lines = list(lines) + ["LIVE preview: adjust camera, then left click to freeze current frame"]
            else:
                lines = list(lines) + ["FROZEN frame: s accept | r resume live"]
            draw_overlay(frame, state["click"], lines, state["valid"], window_title, overlay_scale)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                state["click"] = None
                state["frozen_frame"] = None
            elif key == ord("v"):
                state["valid"] = 0 if state["valid"] else 1
            elif key == ord("s"):
                if state["click"] is None or state["frozen_frame"] is None:
                    print("No frozen click yet. Left-click the target first.")
                    continue
                return state["frozen_frame"], state["click"][0], state["click"][1], state["valid"], f"camera:{camera_ref}({camera_label})"
            elif key == ord("q") or key == 27:
                raise CalibrationError(f"user aborted in window: {window_title}")
    finally:
        cap.release()
        cv2.destroyWindow(window_title)


def preview_camera_until_accept(
    camera_ref: Union[int, str],
    width: int,
    height: int,
    window_title: str,
    line_builder,
    initial_valid: int = 1,
    overlay_scale: float = 1.15,
) -> Tuple[object, int, str]:
    cv2 = import_cv2()
    cap, camera_label = open_camera_capture(camera_ref, width, height)
    valid = int(initial_valid)
    warned_size = False
    last_frame = None

    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise CalibrationError(f"camera frame read failed: {camera_ref} ({camera_label})")
            last_frame = frame
            actual_h, actual_w = frame.shape[:2]
            if not warned_size and (int(actual_w) != int(width) or int(actual_h) != int(height)):
                print(
                    f"[Warn] camera {camera_label} requested {int(width)}x{int(height)}, "
                    f"actual frame is {actual_w}x{actual_h}"
                )
                warned_size = True

            lines = list(line_builder(frame)) + [
                "MANUAL mode: adjust gimbal until target is centered, then press s",
                "v toggle valid | q quit",
            ]
            draw_overlay(frame, None, lines, valid, window_title, overlay_scale)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("v"):
                valid = 0 if valid else 1
            elif key == ord("s"):
                return last_frame.copy(), valid, f"camera:{camera_ref}({camera_label})"
            elif key == ord("q") or key == 27:
                raise CalibrationError(f"user aborted in window: {window_title}")
    finally:
        cap.release()
        cv2.destroyWindow(window_title)


def move_gimbal(ctrl_az: float, ctrl_el: float, port: Optional[str]) -> None:
    if not port:
        raise CalibrationError("--gimbal-port is required for --enable-gimbal-move")

    from gimbal_interface import GT06ZAdapter

    gimbal = GT06ZAdapter(port)
    try:
        if not gimbal.connect():
            raise CalibrationError(f"failed to connect GT06Z gimbal: {port}")
        if not gimbal.wait_ready():
            raise CalibrationError(f"GT06Z gimbal is not ready: {port}")
        print(f"Sending gimbal command: azimuth={ctrl_az:.6f}, elevation={ctrl_el:.6f}")
        gimbal.set_attitude(elevation=ctrl_el, azimuth=ctrl_az)
        input("Press Enter after the gimbal is settled...")
    finally:
        gimbal.close()


def _optional_float(value) -> str:
    return "" if value is None else str(value)


def run_calibration(args) -> CalibrationRecord:
    logic_id, stored_theta_cfg = resolve_camera_params(args.board, args.cam)
    theta_cfg = stored_theta_to_calibration_theta(stored_theta_cfg)
    theta_override_used = args.theta_horizontal is not None or args.theta_vertical is not None
    if theta_override_used:
        theta_cfg = {
            "theta_horizontal": (
                float(args.theta_horizontal)
                if args.theta_horizontal is not None
                else float(theta_cfg["theta_horizontal"])
            ),
            "theta_vertical": (
                float(args.theta_vertical)
                if args.theta_vertical is not None
                else float(theta_cfg["theta_vertical"])
            ),
        }
        print(
            "[Calibration][Warn] Using temporary theta override for this run: "
            f"theta_horizontal={theta_cfg['theta_horizontal']:.6f}, "
            f"theta_vertical={theta_cfg['theta_vertical']:.6f}. "
            "Do not use this record as final DEVICE_THETA calibration unless the camera is in its final mount."
        )
    else:
        print(
            "[Calibration] Using calibration-space theta: "
            f"stored_theta_horizontal={float(stored_theta_cfg['theta_horizontal']):.6f} "
            f"+ GIMBAL_AZ_BASE={GIMBAL_AZ_BASE:.6f} "
            f"=> theta_horizontal={theta_cfg['theta_horizontal']:.6f}, "
            f"theta_vertical={theta_cfg['theta_vertical']:.6f}"
        )
    old_horizontal = float(theta_cfg["theta_horizontal"])
    old_vertical = float(theta_cfg["theta_vertical"])

    def lower_lines_for_frame(frame, click):
        lower_h_current, lower_w_current = frame.shape[:2]
        lines = [
            f"board={args.board} cam={args.cam} logic_id={logic_id}",
            f"theta_h={old_horizontal:.6f} theta_v={old_vertical:.6f}",
            f"image={lower_w_current}x{lower_h_current} fov={FOV_X:.6f}x{FOV_Y:.6f}",
        ]
        if click is not None:
            result = compute_camera_angle_from_click(
                click[0],
                click[1],
                lower_w_current,
                lower_h_current,
                theta_cfg,
            )
            ctrl_az, ctrl_el = calibration_ui_to_ctrl_angles(result["calc_ui_az"], result["calc_ui_el"])
            lines.extend([
                f"dx={result['dx']:.2f} dy={result['dy']:.2f}",
                f"offset_az={result['offset_az']:.6f} offset_el={result['offset_el']:.6f}",
                f"calc_ui_az={result['calc_ui_az']:.6f} calc_ui_el={result['calc_ui_el']:.6f}",
                f"ctrl_az={ctrl_az:.6f} ctrl_el={ctrl_el:.6f}",
            ])
        return lines

    if args.mode == "camera":
        if args.lower_camera is None:
            raise CalibrationError("--lower-camera is required in camera mode")
        lower_frame, lower_x, lower_y, valid, lower_source = interactive_camera_click(
            args.lower_camera,
            args.lower_camera_w,
            args.lower_camera_h,
            "lower camera calibration",
            lower_lines_for_frame,
            args.valid,
            args.overlay_scale,
        )
    else:
        lower_frame, lower_source = load_frame_for_phase(args, "lower")

        def lower_lines(click):
            return lower_lines_for_frame(lower_frame, click)

        lower_x, lower_y, valid = interactive_click(
            lower_frame,
            "lower camera calibration",
            lower_lines,
            args.valid,
            args.overlay_scale,
        )

    lower_h, lower_w = lower_frame.shape[:2]
    lower_result = compute_camera_angle_from_click(lower_x, lower_y, lower_w, lower_h, theta_cfg)
    ctrl_az, ctrl_el = calibration_ui_to_ctrl_angles(lower_result["calc_ui_az"], lower_result["calc_ui_el"])

    print(
        f"calc_ui_az={lower_result['calc_ui_az']:.6f}, "
        f"calc_ui_el={lower_result['calc_ui_el']:.6f}, "
        f"ctrl_az={ctrl_az:.6f}, ctrl_el={ctrl_el:.6f}"
    )

    if args.enable_gimbal_move:
        move_gimbal(ctrl_az, ctrl_el, args.gimbal_port)
    else:
        print("Gimbal move disabled. Manually point the gimbal to the printed control angles if needed.")

    gimbal_data = {
        "w": "",
        "h": "",
        "x": "",
        "y": "",
        "dx": "",
        "dy": "",
        "offset_az": "",
        "offset_el": "",
    }
    manual_data = {
        "true_ctrl_az": "",
        "true_ctrl_el": "",
        "true_ui_az": "",
        "true_ui_el": "",
    }

    if args.manual_true_angle:
        print(
            "[Manual] manual true-angle mode enabled. "
            "The gimbal camera preview will open next; manually adjust the gimbal until the target is centered, "
            "then press 's' in the preview window. After that, enter true_ctrl_az and true_ctrl_el in this terminal."
        )

        def manual_gimbal_lines_for_frame(frame):
            gimbal_h_current, gimbal_w_current = frame.shape[:2]
            return [
                f"gimbal image={gimbal_w_current}x{gimbal_h_current}",
                f"calc_ui_az={lower_result['calc_ui_az']:.6f} calc_ui_el={lower_result['calc_ui_el']:.6f}",
                f"ctrl_az={ctrl_az:.6f} ctrl_el={ctrl_el:.6f}",
            ]

        if args.mode == "camera":
            if args.gimbal_camera is None:
                raise CalibrationError("--gimbal-camera is required in manual true-angle camera mode")
            gimbal_frame, valid, gimbal_source = preview_camera_until_accept(
                args.gimbal_camera,
                args.gimbal_camera_w,
                args.gimbal_camera_h,
                "gimbal camera manual true-angle preview",
                manual_gimbal_lines_for_frame,
                valid,
                args.overlay_scale,
            )
            gimbal_h, gimbal_w = gimbal_frame.shape[:2]
            gimbal_data = {
                "w": gimbal_w,
                "h": gimbal_h,
                "x": "",
                "y": "",
                "dx": "",
                "dy": "",
                "offset_az": "",
                "offset_el": "",
            }
        else:
            gimbal_source = ""

        true_ctrl_az = args.true_ctrl_az
        true_ctrl_el = args.true_ctrl_el
        if true_ctrl_az is None:
            print("[Manual] Enter the real gimbal azimuth after centering the target.")
            true_ctrl_az = float(input("true_ctrl_az: ").strip())
        if true_ctrl_el is None:
            print("[Manual] Enter the real gimbal elevation after centering the target.")
            true_ctrl_el = float(input("true_ctrl_el: ").strip())
        delta_data = compute_delta_by_manual_true_angle(
            lower_result["calc_ui_az"],
            lower_result["calc_ui_el"],
            true_ctrl_az,
            true_ctrl_el,
        )
        manual_data = {
            "true_ctrl_az": true_ctrl_az,
            "true_ctrl_el": true_ctrl_el,
            "true_ui_az": delta_data["true_ui_az"],
            "true_ui_el": delta_data["true_ui_el"],
        }
        delta_az = delta_data["delta_az"]
        delta_el = delta_data["delta_el"]
        gimbal_source = ""
    else:
        if args.gimbal_cam_fov_x is None or args.gimbal_cam_fov_y is None:
            raise CalibrationError(
                "--gimbal-cam-fov-x and --gimbal-cam-fov-y are required unless --manual-true-angle is used"
            )
        def gimbal_lines_for_frame(frame, click):
            gimbal_h_current, gimbal_w_current = frame.shape[:2]
            lines = [
                f"gimbal image={gimbal_w_current}x{gimbal_h_current}",
                f"gimbal fov={args.gimbal_cam_fov_x:.6f}x{args.gimbal_cam_fov_y:.6f}",
                f"calc_ui_az={lower_result['calc_ui_az']:.6f} calc_ui_el={lower_result['calc_ui_el']:.6f}",
                f"ctrl_az={ctrl_az:.6f} ctrl_el={ctrl_el:.6f}",
            ]
            if click is not None:
                result = compute_gimbal_offset_from_click(
                    click[0],
                    click[1],
                    gimbal_w_current,
                    gimbal_h_current,
                    args.gimbal_cam_fov_x,
                    args.gimbal_cam_fov_y,
                )
                lines.extend([
                    f"dx={result['dx']:.2f} dy={result['dy']:.2f}",
                    f"delta_az={result['offset_az']:.6f} delta_el={result['offset_el']:.6f}",
                ])
            return lines

        if args.mode == "camera":
            if args.gimbal_camera is None:
                raise CalibrationError("--gimbal-camera is required unless --manual-true-angle is used")
            gimbal_frame, gimbal_x, gimbal_y, valid, gimbal_source = interactive_camera_click(
                args.gimbal_camera,
                args.gimbal_camera_w,
                args.gimbal_camera_h,
                "gimbal camera calibration",
                gimbal_lines_for_frame,
                valid,
                args.overlay_scale,
            )
        else:
            gimbal_frame, gimbal_source = load_frame_for_phase(args, "gimbal")

            def gimbal_lines(click):
                return gimbal_lines_for_frame(gimbal_frame, click)

            gimbal_x, gimbal_y, valid = interactive_click(
                gimbal_frame,
                "gimbal camera calibration",
                gimbal_lines,
                valid,
                args.overlay_scale,
            )

        gimbal_h, gimbal_w = gimbal_frame.shape[:2]
        if args.gimbal_cam_w and int(args.gimbal_cam_w) != int(gimbal_w):
            print(f"[Warn] --gimbal-cam-w={args.gimbal_cam_w} differs from actual frame width={gimbal_w}")
        if args.gimbal_cam_h and int(args.gimbal_cam_h) != int(gimbal_h):
            print(f"[Warn] --gimbal-cam-h={args.gimbal_cam_h} differs from actual frame height={gimbal_h}")

        gimbal_result = compute_gimbal_offset_from_click(
            gimbal_x,
            gimbal_y,
            gimbal_w,
            gimbal_h,
            args.gimbal_cam_fov_x,
            args.gimbal_cam_fov_y,
        )
        delta_az = gimbal_result["offset_az"]
        delta_el = gimbal_result["offset_el"]
        gimbal_data = {
            "w": gimbal_w,
            "h": gimbal_h,
            "x": gimbal_x,
            "y": gimbal_y,
            "dx": gimbal_result["dx"],
            "dy": gimbal_result["dy"],
            "offset_az": gimbal_result["offset_az"],
            "offset_el": gimbal_result["offset_el"],
        }

    new_theta_horizontal = (old_horizontal + delta_az) % 360.0
    new_theta_vertical = old_vertical + delta_el

    note = args.note or ""
    if theta_override_used:
        override_note = "temporary_theta_override_not_final_mount"
        note = f"{note};{override_note}" if note else override_note
    basis_note = "theta_basis=calibration_space_DEVICE_THETA_plus_GIMBAL_AZ_BASE;ctrl_conversion=no_base_add"
    note = f"{note};{basis_note}" if note else basis_note

    record = CalibrationRecord(
        timestamp=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        board=args.board,
        cam=int(args.cam),
        logic_id=int(logic_id),
        target_name=args.target_name or "",
        distance_est=_optional_float(args.distance_est),
        mode=args.mode,
        lower_image_path=lower_source,
        gimbal_image_path=gimbal_source,
        lower_w=int(lower_w),
        lower_h=int(lower_h),
        lower_x=float(lower_x),
        lower_y=float(lower_y),
        lower_dx=lower_result["dx"],
        lower_dy=lower_result["dy"],
        lower_offset_az=lower_result["offset_az"],
        lower_offset_el=lower_result["offset_el"],
        old_theta_horizontal=old_horizontal,
        old_theta_vertical=old_vertical,
        calc_ui_az=lower_result["calc_ui_az"],
        calc_ui_el=lower_result["calc_ui_el"],
        ctrl_az=ctrl_az,
        ctrl_el=ctrl_el,
        gimbal_w=gimbal_data["w"],
        gimbal_h=gimbal_data["h"],
        gimbal_x=gimbal_data["x"],
        gimbal_y=gimbal_data["y"],
        gimbal_dx=gimbal_data["dx"],
        gimbal_dy=gimbal_data["dy"],
        gimbal_cam_fov_x=_optional_float(args.gimbal_cam_fov_x),
        gimbal_cam_fov_y=_optional_float(args.gimbal_cam_fov_y),
        gimbal_offset_az=gimbal_data["offset_az"],
        gimbal_offset_el=gimbal_data["offset_el"],
        manual_true_angle=1 if args.manual_true_angle else 0,
        true_ctrl_az=manual_data["true_ctrl_az"],
        true_ctrl_el=manual_data["true_ctrl_el"],
        true_ui_az=manual_data["true_ui_az"],
        true_ui_el=manual_data["true_ui_el"],
        delta_az=delta_az,
        delta_el=delta_el,
        new_theta_horizontal=new_theta_horizontal,
        new_theta_vertical=new_theta_vertical,
        valid=int(valid),
        note=note,
    )

    append_record_csv(args.record_csv, record)
    print(f"delta_az={delta_az:.6f}, delta_el={delta_el:.6f}")
    print(f"suggested theta_horizontal={new_theta_horizontal:.6f}, theta_vertical={new_theta_vertical:.6f}")
    print(f"Appended {args.record_csv}")
    return record


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate lower camera matrix angles against the gimbal-mounted camera."
    )
    parser.add_argument("--summarize", help="Summarize a calibration_records.csv file and exit.")
    parser.add_argument("--summary-csv", default="calibration_summary.csv")
    parser.add_argument("--suggested-output", default="suggested_DEVICE_THETA.py")

    parser.add_argument("--board", default=DEFAULT_BOARD)
    parser.add_argument("--cam", type=int, default=DEFAULT_CAM)
    parser.add_argument("--mode", choices=("image", "camera"), default=DEFAULT_MODE)
    parser.add_argument("--lower-image")
    parser.add_argument("--gimbal-image")
    parser.add_argument(
        "--lower-camera",
        default=DEFAULT_LOWER_CAMERA,
        help="DirectShow camera name or OpenCV index, e.g. 000000002, 2, dshow:2, or msmf:2",
    )
    parser.add_argument(
        "--gimbal-camera",
        default=DEFAULT_GIMBAL_CAMERA,
        help="DirectShow camera name or OpenCV index, e.g. 000000008, 1, dshow:1, or msmf:1",
    )
    parser.add_argument("--lower-camera-w", type=int, default=int(IMG_W))
    parser.add_argument("--lower-camera-h", type=int, default=int(IMG_H))
    parser.add_argument("--gimbal-camera-w", type=int, default=int(IMG_W))
    parser.add_argument("--gimbal-camera-h", type=int, default=int(IMG_H))
    parser.add_argument("--target-name", default=DEFAULT_TARGET_NAME)
    parser.add_argument("--distance-est", type=float)
    parser.add_argument("--record-csv", default=DEFAULT_RECORD_CSV)
    parser.add_argument("--valid", type=int, choices=(0, 1), default=1)
    parser.add_argument("--note", default="")
    parser.add_argument("--overlay-scale", type=float, default=DEFAULT_OVERLAY_SCALE, help="OpenCV overlay text scale; increase for 4K displays.")
    parser.add_argument(
        "--theta-horizontal",
        type=float,
        help=(
            "Temporary calibration-space horizontal theta override for workflow tests "
            "(already includes GIMBAL_AZ_BASE); do not use for final calibration records."
        ),
    )
    parser.add_argument(
        "--theta-vertical",
        type=float,
        help="Temporary calibration-space vertical theta override for workflow tests; do not use for final calibration records.",
    )

    parser.add_argument("--gimbal-cam-w", type=int)
    parser.add_argument("--gimbal-cam-h", type=int)
    parser.add_argument("--gimbal-cam-fov-x", type=float, default=DEFAULT_GIMBAL_CAM_FOV_X)
    parser.add_argument("--gimbal-cam-fov-y", type=float, default=DEFAULT_GIMBAL_CAM_FOV_Y)
    parser.add_argument("--manual-true-angle", dest="manual_true_angle", action="store_true")
    parser.add_argument("--no-manual-true-angle", dest="manual_true_angle", action="store_false")
    parser.set_defaults(manual_true_angle=DEFAULT_MANUAL_TRUE_ANGLE)
    parser.add_argument("--true-ctrl-az", type=float)
    parser.add_argument("--true-ctrl-el", type=float)

    parser.add_argument("--enable-gimbal-move", dest="enable_gimbal_move", action="store_true")
    parser.add_argument("--no-enable-gimbal-move", dest="enable_gimbal_move", action="store_false")
    parser.set_defaults(enable_gimbal_move=DEFAULT_ENABLE_GIMBAL_MOVE)
    parser.add_argument("--gimbal-port", default=DEFAULT_GIMBAL_PORT)
    return parser


def validate_args(args) -> None:
    if args.summarize:
        return
    missing = []
    if not args.board:
        missing.append("--board")
    if args.cam is None:
        missing.append("--cam")
    if missing:
        raise CalibrationError("missing required arguments: " + ", ".join(missing))
    if args.manual_true_angle and ((args.true_ctrl_az is None) ^ (args.true_ctrl_el is None)):
        raise CalibrationError("--true-ctrl-az and --true-ctrl-el must be provided together")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        if args.summarize:
            summarize_records(args.summarize, args.summary_csv, args.suggested_output)
        else:
            run_calibration(args)
        return 0
    except CalibrationError as exc:
        print(f"[Calibration][Error] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
