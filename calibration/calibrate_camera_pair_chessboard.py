#!/usr/bin/env python3
"""Estimate the raw angle bias between a gimbal camera and one matrix camera."""

from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from GT06Z_gimbal import GT06ZGimbal


DEFAULT_FOV_X = 17.5
DEFAULT_FOV_Y = 9.9
PATTERN_SIZE = (15, 11)


@dataclass
class Sample:
    ts: float
    ctrl_az: float
    ctrl_el: float
    gimbal_cx: float
    gimbal_cy: float
    down_cx: float
    down_cy: float
    chessboard_az: float
    chessboard_el: float
    bias_az: float
    bias_el: float
    relative_az: float
    relative_el: float
    offset_az_g: float
    offset_el_g: float
    offset_az_d: float
    offset_el_d: float


def norm360(angle: float) -> float:
    return angle % 360.0


def signed_angle_diff(a: float, b: float) -> float:
    return ((a - b + 180.0) % 360.0) - 180.0


def circular_mean_deg(values: list[float]) -> float:
    if not values:
        return float("nan")
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    return norm360(math.degrees(math.atan2(sin_sum, cos_sum)))


def circular_std_deg(values: list[float], mean_deg: float) -> float:
    if len(values) < 2:
        return 0.0
    diffs = [signed_angle_diff(v, mean_deg) for v in values]
    return float(np.std(diffs, ddof=1))


def parse_camera_source(value: str) -> int | str:
    stripped = str(value).strip()
    if re.fullmatch(r"[1-9]\d*|0", stripped):
        return int(stripped)
    return stripped


def list_dshow_video_devices() -> list[str]:
    try:
        proc = subprocess.run(
            ["ffmpeg", "-hide_banner", "-f", "dshow", "-list_devices", "true", "-i", "dummy"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=10,
        )
    except Exception:
        return []

    text = (proc.stdout or "") + "\n" + (proc.stderr or "")
    devices: list[str] = []
    for line in text.splitlines():
        match = re.search(r'\] "(.+)" \(video\)', line)
        if match:
            devices.append(match.group(1))
    return devices


def resolve_camera_source(source: int | str) -> int:
    if isinstance(source, int):
        return source

    devices = list_dshow_video_devices()
    if not devices:
        raise RuntimeError(f"Cannot resolve camera name {source!r}: ffmpeg dshow device list is unavailable.")

    for index, name in enumerate(devices):
        if name == source:
            return index

    available = ", ".join(f"{i}:{name}" for i, name in enumerate(devices))
    raise RuntimeError(f"Camera name {source!r} was not found. Available dshow devices: {available}")


def open_camera(source: int | str, width: int | None, height: int | None) -> tuple[cv2.VideoCapture, int]:
    index = resolve_camera_source(source)
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index={index} source={source!r}")
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap, index


def scan_cameras(max_index: int) -> None:
    devices = list_dshow_video_devices()
    if devices:
        print("=== DirectShow Video Devices ===")
        for index, name in enumerate(devices):
            print(f"name={name}, mapped OpenCV index={index}")

    print("=== OpenCV Camera Scan ===")
    for index in range(max_index):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        opened = cap.isOpened()
        read_ok = False
        width = 0
        height = 0
        if opened:
            read_ok, frame = cap.read()
            if read_ok:
                height, width = frame.shape[:2]
            else:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        name = devices[index] if index < len(devices) else ""
        suffix = f", name={name}" if name else ""
        print(f"index={index}: opened={opened}, read={read_ok}, size={width}x{height}{suffix}")


def check_hardware(args: argparse.Namespace) -> None:
    print("[Check] opening gimbal serial...")
    gimbal = GT06ZGimbal(args.gimbal_port)
    if not gimbal.open():
        raise RuntimeError(f"Cannot open gimbal serial port {args.gimbal_port}")

    cap_gimbal = None
    cap_down = None
    try:
        print("[Check] opening gimbal camera...")
        cap_gimbal, gimbal_index = open_camera(args.gimbal_camera, args.width, args.height)
        ok_g, frame_g = cap_gimbal.read()
        if not ok_g:
            raise RuntimeError(f"Gimbal camera source={args.gimbal_camera!r} opened but frame read failed")
        print(
            f"[Check] gimbal camera source={args.gimbal_camera!r}, "
            f"index={gimbal_index}, frame={frame_g.shape[1]}x{frame_g.shape[0]}"
        )

        print("[Check] opening down camera...")
        cap_down, down_index = open_camera(args.down_camera, args.width, args.height)
        ok_d, frame_d = cap_down.read()
        if not ok_d:
            raise RuntimeError(f"Down camera source={args.down_camera!r} opened but frame read failed")
        print(
            f"[Check] down camera source={args.down_camera!r}, "
            f"index={down_index}, frame={frame_d.shape[1]}x{frame_d.shape[0]}"
        )

        result = gimbal.query_angles()
        if result is None:
            print("[Check][Warn] gimbal serial opened, but angle feedback was not read")
        else:
            ctrl_el, ctrl_az = result
            print(f"[Check] gimbal attitude ctrl_az={ctrl_az:.2f}, ctrl_el={ctrl_el:.2f}")
        print("[Check] OK")
    finally:
        if cap_gimbal is not None:
            cap_gimbal.release()
        if cap_down is not None:
            cap_down.release()
        gimbal.close()


def detect_chessboard(frame: np.ndarray) -> tuple[bool, np.ndarray | None, tuple[float, float] | None]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    variants: list[tuple[np.ndarray, float]] = [(gray, 1.0)]

    # Rolling-shutter/projector flicker often appears as horizontal bands.
    # Equalize each row's median brightness before chessboard detection.
    row_median = np.median(gray, axis=1).astype(np.float32)
    row_bias = cv2.GaussianBlur(row_median.reshape(-1, 1), (1, 31), 0).reshape(-1)
    band_corrected = gray.astype(np.float32) - row_bias[:, None] + float(np.median(row_bias))
    band_corrected = np.clip(band_corrected, 0, 255).astype(np.uint8)
    variants.append((band_corrected, 1.0))

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    variants.append((clahe, 1.0))
    band_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(band_corrected)
    variants.append((band_clahe, 1.0))

    blurred = cv2.GaussianBlur(gray, (0, 0), 1.2)
    sharpened = cv2.addWeighted(gray, 1.8, blurred, -0.8, 0)
    variants.append((sharpened, 1.0))

    h, w = gray.shape[:2]
    if max(w, h) > 900:
        scale = 0.5
        small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        variants.append((small, scale))
        small_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(small)
        variants.append((small_clahe, scale))

    if hasattr(cv2, "findChessboardCornersSB"):
        sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        if hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
            sb_flags |= cv2.CALIB_CB_EXHAUSTIVE
        if hasattr(cv2, "CALIB_CB_ACCURACY"):
            sb_flags |= cv2.CALIB_CB_ACCURACY

        for img, scale in variants:
            ok, corners = cv2.findChessboardCornersSB(img, PATTERN_SIZE, sb_flags)
            if ok:
                if scale != 1.0:
                    corners = corners / scale
                center = np.mean(corners.reshape(-1, 2), axis=0)
                return True, corners, (float(center[0]), float(center[1]))

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if hasattr(cv2, "CALIB_CB_FILTER_QUADS"):
        flags |= cv2.CALIB_CB_FILTER_QUADS
    for img, scale in variants:
        ok, corners = cv2.findChessboardCorners(img, PATTERN_SIZE, flags)
        if ok:
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
            if scale != 1.0:
                corners = corners / scale
            center = np.mean(corners.reshape(-1, 2), axis=0)
            return True, corners, (float(center[0]), float(center[1]))

    return False, None, None


def draw_detection(
    frame: np.ndarray,
    found: bool,
    corners: np.ndarray | None,
    center: tuple[float, float] | None,
    title: str,
    footer: str,
    message: str,
) -> np.ndarray:
    view = frame.copy()
    h, w = view.shape[:2]
    cv2.line(view, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 255), 1)
    cv2.line(view, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 255), 1)
    if found and corners is not None and center is not None:
        cv2.drawChessboardCorners(view, PATTERN_SIZE, corners, found)
        cx, cy = int(round(center[0])), int(round(center[1]))
        cv2.circle(view, (cx, cy), 8, (0, 0, 255), 2)
        status = f"{title}: FOUND center=({center[0]:.1f}, {center[1]:.1f})"
        color = (0, 220, 0)
    else:
        status = f"{title}: NOT FOUND"
        color = (0, 0, 255)
    cv2.putText(view, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
    cv2.putText(view, message, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(view, footer, (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2, cv2.LINE_AA)
    return view


def stable_center(history: deque[tuple[float, float] | None], min_found: int) -> tuple[float, float] | None:
    centers = [center for center in history if center is not None]
    if len(centers) < min_found:
        return None
    arr = np.array(centers, dtype=np.float32)
    median = np.median(arr, axis=0)
    return float(median[0]), float(median[1])


def query_gimbal_angles(gimbal: GT06ZGimbal, retries: int = 5) -> tuple[float, float]:
    for _ in range(retries):
        result = gimbal.query_angles()
        if result is not None:
            ctrl_el, ctrl_az = result
            return float(ctrl_az), float(ctrl_el)
        time.sleep(0.08)
    raise RuntimeError("Failed to read gimbal attitude")


def compute_sample(
    ctrl_az: float,
    ctrl_el: float,
    gimbal_center: tuple[float, float],
    down_center: tuple[float, float],
    gimbal_shape: tuple[int, int],
    down_shape: tuple[int, int],
    gimbal_fov_x: float,
    gimbal_fov_y: float,
    down_fov_x: float,
    down_fov_y: float,
) -> Sample:
    g_h, g_w = gimbal_shape
    d_h, d_w = down_shape
    g_cx, g_cy = gimbal_center
    d_cx, d_cy = down_center

    # Viewpoint convention:
    # Describe directions from behind/inside the gimbal camera looking outward.
    # Image-right is positive azimuth, image-up is positive elevation.
    # Do not mirror these signs as if standing face-to-face with the camera.
    offset_az_g = (g_cx - g_w / 2.0) * (gimbal_fov_x / g_w)
    offset_el_g = -(g_cy - g_h / 2.0) * (gimbal_fov_y / g_h)
    chessboard_az = norm360(ctrl_az + offset_az_g)
    chessboard_el = ctrl_el + offset_el_g

    offset_az_d = (d_cx - d_w / 2.0) * (down_fov_x / d_w)
    offset_el_d = -(d_cy - d_h / 2.0) * (down_fov_y / d_h)
    bias_az = norm360(chessboard_az - offset_az_d)
    bias_el = chessboard_el - offset_el_d

    return Sample(
        ts=time.time(),
        ctrl_az=ctrl_az,
        ctrl_el=ctrl_el,
        gimbal_cx=g_cx,
        gimbal_cy=g_cy,
        down_cx=d_cx,
        down_cy=d_cy,
        chessboard_az=chessboard_az,
        chessboard_el=chessboard_el,
        bias_az=bias_az,
        bias_el=bias_el,
        relative_az=signed_angle_diff(bias_az, ctrl_az),
        relative_el=bias_el - ctrl_el,
        offset_az_g=offset_az_g,
        offset_el_g=offset_el_g,
        offset_az_d=offset_az_d,
        offset_el_d=offset_el_d,
    )


def save_samples(path: Path, samples: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(Sample.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for sample in samples:
            writer.writerow({field: getattr(sample, field) for field in fields})


def print_summary(samples: list[Sample], camera_label: str) -> None:
    bias_az_values = [s.bias_az for s in samples]
    bias_el_values = [s.bias_el for s in samples]
    relative_az_values = [s.relative_az for s in samples]
    relative_el_values = [s.relative_el for s in samples]

    bias_az_mean = circular_mean_deg(bias_az_values)
    bias_el_mean = float(np.mean(bias_el_values))
    relative_az_mean = float(np.mean(relative_az_values))
    relative_el_mean = float(np.mean(relative_el_values))

    bias_az_std = circular_std_deg(bias_az_values, bias_az_mean)
    bias_el_std = float(np.std(bias_el_values, ddof=1)) if len(bias_el_values) > 1 else 0.0
    relative_az_std = float(np.std(relative_az_values, ddof=1)) if len(relative_az_values) > 1 else 0.0
    relative_el_std = float(np.std(relative_el_values, ddof=1)) if len(relative_el_values) > 1 else 0.0

    print("\n=== Calibration Result ===")
    print(f"camera_id: {camera_label}")
    print(f"samples: {len(samples)}")
    print("\n[Raw matrix-camera bias angle in gimbal control coordinates]")
    print(f"bias_horizontal_mean: {bias_az_mean:.4f}")
    print(f"bias_vertical_mean:   {bias_el_mean:.4f}")
    print(f"bias_std_az: {bias_az_std:.4f} deg")
    print(f"bias_std_el: {bias_el_std:.4f} deg")
    print("\n[Relative turn from current gimbal attitude to this bias angle]")
    print(f"relative_turn_az_mean: {relative_az_mean:.4f} deg")
    print(f"relative_turn_el_mean: {relative_el_mean:.4f} deg")
    print(f"relative_turn_az_std: {relative_az_std:.4f} deg")
    print(f"relative_turn_el_std: {relative_el_std:.4f} deg")
    print("\nUse bias_* as the raw calibration result. Baseline normalization is a separate later step.")


def make_synthetic_chessboard(cols: int = 16, rows: int = 12, square_px: int = 48, margin_px: int = 80) -> np.ndarray:
    width = cols * square_px + 2 * margin_px
    height = rows * square_px + 2 * margin_px
    gray = np.full((height, width), 255, dtype=np.uint8)
    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                x0 = margin_px + col * square_px
                y0 = margin_px + row * square_px
                gray[y0 : y0 + square_px, x0 : x0 + square_px] = 0
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def run_self_test() -> None:
    frame = make_synthetic_chessboard()
    found, corners, center = detect_chessboard(frame)
    if not found or corners is None or center is None:
        raise RuntimeError("self-test failed: synthetic chessboard was not detected")

    sample = compute_sample(
        ctrl_az=60.0,
        ctrl_el=2.0,
        gimbal_center=center,
        down_center=center,
        gimbal_shape=frame.shape[:2],
        down_shape=frame.shape[:2],
        gimbal_fov_x=DEFAULT_FOV_X,
        gimbal_fov_y=DEFAULT_FOV_Y,
        down_fov_x=DEFAULT_FOV_X,
        down_fov_y=DEFAULT_FOV_Y,
    )
    if not (0.0 <= sample.bias_az < 360.0) or not math.isfinite(sample.bias_el):
        raise RuntimeError("self-test failed: computed bias angles are invalid")

    print("[SelfTest] OK")
    print(f"[SelfTest] detected center=({center[0]:.2f}, {center[1]:.2f})")
    print(f"[SelfTest] computed bias=({sample.bias_az:.4f}, {sample.bias_el:.4f})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate raw camera bias angle with a chessboard.")
    parser.add_argument("--self-test", action="store_true", help="run a non-hardware synthetic chessboard test and exit")
    parser.add_argument("--list-cameras", action="store_true", help="scan OpenCV camera indices and exit")
    parser.add_argument("--check-hardware", action="store_true", help="open gimbal serial and both cameras, read one frame, then exit")
    parser.add_argument("--scan-max-index", type=int, default=10, help="max camera index used by --list-cameras")
    parser.add_argument("--gimbal-camera", type=parse_camera_source, default="000000008", help="OpenCV index or DirectShow name for gimbal camera")
    parser.add_argument("--down-camera", type=parse_camera_source, default="000000005", help="OpenCV index or DirectShow name for matrix camera")
    parser.add_argument("--down-camera-label", default="000000005", help="label printed in the final result")
    parser.add_argument("--gimbal-port", default="COM11", help="GT06Z serial port")
    parser.add_argument("--width", type=int, default=None, help="optional capture width for both cameras")
    parser.add_argument("--height", type=int, default=None, help="optional capture height for both cameras")
    parser.add_argument("--samples", type=int, default=10, help="number of samples to collect before auto exit")
    parser.add_argument("--auto-window", type=int, default=20, help="rolling frame window used for automatic sampling")
    parser.add_argument("--auto-min-found", type=int, default=8, help="minimum found frames inside the rolling window")
    parser.add_argument("--sample-interval", type=float, default=1.0, help="minimum seconds between automatic samples")
    parser.add_argument("--debug-keys", action="store_true", help="print OpenCV key codes received by waitKeyEx")
    parser.add_argument("--gimbal-fov-x", type=float, default=DEFAULT_FOV_X)
    parser.add_argument("--gimbal-fov-y", type=float, default=DEFAULT_FOV_Y)
    parser.add_argument("--down-fov-x", type=float, default=DEFAULT_FOV_X)
    parser.add_argument("--down-fov-y", type=float, default=DEFAULT_FOV_Y)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("logs") / "camera_pair_chessboard_calibration.csv",
        help="CSV file for raw samples",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return
    if args.list_cameras:
        scan_cameras(args.scan_max_index)
        return
    if args.check_hardware:
        check_hardware(args)
        return

    gimbal = GT06ZGimbal(args.gimbal_port)
    cap_gimbal = None
    cap_down = None
    samples: list[Sample] = []

    print(f"[Init] pattern_size={PATTERN_SIZE}, gimbal_camera={args.gimbal_camera}, down_camera={args.down_camera}")
    print(f"[Init] gimbal_port={args.gimbal_port}")
    print("[Keys] Q/Esc=quit. Sampling is automatic.")

    try:
        if not gimbal.open():
            raise RuntimeError(f"Cannot open gimbal serial port {args.gimbal_port}")

        cap_gimbal, gimbal_index = open_camera(args.gimbal_camera, args.width, args.height)
        cap_down, down_index = open_camera(args.down_camera, args.width, args.height)
        print(f"[Init] resolved gimbal_camera index={gimbal_index}, down_camera index={down_index}")

        history_g: deque[tuple[float, float] | None] = deque(maxlen=args.auto_window)
        history_d: deque[tuple[float, float] | None] = deque(maxlen=args.auto_window)
        next_sample_time = time.time() + 0.5
        last_message = "auto sampling: keep both chessboards visible; Q/Esc to quit"

        while True:
            ok_g, frame_g = cap_gimbal.read()
            ok_d, frame_d = cap_down.read()
            if not ok_g or not ok_d:
                print("[Warn] camera frame read failed")
                time.sleep(0.05)
                continue

            found_g, corners_g, center_g = detect_chessboard(frame_g)
            found_d, corners_d, center_d = detect_chessboard(frame_d)
            history_g.append(center_g if found_g and center_g is not None else None)
            history_d.append(center_d if found_d and center_d is not None else None)

            stable_g = stable_center(history_g, args.auto_min_found)
            stable_d = stable_center(history_d, args.auto_min_found)
            found_count_g = sum(center is not None for center in history_g)
            found_count_d = sum(center is not None for center in history_d)

            footer = (
                f"samples {len(samples)}/{args.samples} | "
                f"stable G/D {found_count_g}/{args.auto_min_found}, {found_count_d}/{args.auto_min_found} | "
                "Q/Esc=quit"
            )
            view_g = draw_detection(frame_g, found_g, corners_g, center_g, "GIMBAL", footer, last_message)
            view_d = draw_detection(frame_d, found_d, corners_d, center_d, "MATRIX", footer, last_message)

            cv2.imshow("gimbal camera", view_g)
            cv2.imshow("matrix camera", view_d)
            key = cv2.waitKeyEx(30)

            if key != -1 and args.debug_keys:
                print(f"[Key] code={key}")
            if key in (ord("q"), ord("Q"), 27):
                break

            now = time.time()
            if stable_g is not None and stable_d is not None and now >= next_sample_time:
                ctrl_az, ctrl_el = query_gimbal_angles(gimbal)
                sample = compute_sample(
                    ctrl_az=ctrl_az,
                    ctrl_el=ctrl_el,
                    gimbal_center=stable_g,
                    down_center=stable_d,
                    gimbal_shape=frame_g.shape[:2],
                    down_shape=frame_d.shape[:2],
                    gimbal_fov_x=args.gimbal_fov_x,
                    gimbal_fov_y=args.gimbal_fov_y,
                    down_fov_x=args.down_fov_x,
                    down_fov_y=args.down_fov_y,
                )
                samples.append(sample)
                last_message = f"sampled {len(samples)}/{args.samples}: bias=({sample.bias_az:.3f}, {sample.bias_el:.3f})"
                next_sample_time = now + args.sample_interval
                print(
                    f"[Sample {len(samples)}/{args.samples}] "
                    f"ctrl=({sample.ctrl_az:.2f}, {sample.ctrl_el:.2f}), "
                    f"chessboard=({sample.chessboard_az:.3f}, {sample.chessboard_el:.3f}), "
                    f"bias=({sample.bias_az:.3f}, {sample.bias_el:.3f}), "
                    f"relative=({sample.relative_az:.3f}, {sample.relative_el:.3f}), "
                    f"down_offset=({sample.offset_az_d:.3f}, {sample.offset_el_d:.3f})"
                )
                if len(samples) >= args.samples:
                    break
            elif now >= next_sample_time:
                last_message = (
                    f"waiting stable detection: G={found_count_g}/{args.auto_min_found}, "
                    f"D={found_count_d}/{args.auto_min_found}"
                )

        if samples:
            save_samples(args.output, samples)
            print_summary(samples, args.down_camera_label)
            print(f"\nraw_samples_csv: {args.output}")
        else:
            print("[Exit] no samples collected")

    finally:
        if cap_gimbal is not None:
            cap_gimbal.release()
        if cap_down is not None:
            cap_down.release()
        cv2.destroyAllWindows()
        gimbal.close()


if __name__ == "__main__":
    main()
