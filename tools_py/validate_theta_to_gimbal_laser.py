"""
validate_theta_to_gimbal_laser.py usage

Mode 1: lower-camera theta validation + manual laser reading
Command:
    python validate_theta_to_gimbal_laser.py
Flow:
    1. Open the lower camera.
    2. Click the target in the lower-camera view, then press s.
    3. Use DEFAULT_THETA_HORIZONTAL / DEFAULT_THETA_VERTICAL and FOV_X/FOV_Y
       to convert the clicked point to gimbal control angles.
    4. Move the gimbal to ctrl_az / ctrl_el and wait until settled.
    5. Release the gimbal serial port.
    6. Open the gimbal-camera view.
    7. Press l to start continuous SDDM laser reading while the view stays open.
    8. Press q or Esc to stop laser reading and close the view.

Mode 2: lower-camera theta validation + automatic laser grid scan
Command:
    python validate_theta_to_gimbal_laser.py --enable-laser-scan
Flow:
    1. Same lower-camera target selection and theta conversion as Mode 1.
    2. Move the gimbal camera center to the selected target and wait until settled.
    3. Start an automatic scan around that center angle:
           scan_az = ctrl_az + az_offset
           scan_el = ctrl_el + el_offset
    4. Keep showing the gimbal-camera view while moving, settling, and sampling.
    5. Write every scan point to laser_scan_calibration.csv by default.
    6. Scan the full grid unless --scan-stop-on-first-hit is provided.
    7. Print the best LASER_COMP_AZ / LASER_COMP_EL when a hit is found.

Mode 3: gimbal-camera-only laser-axis calibration
Command:
    python validate_theta_to_gimbal_laser.py --gimbal-camera-laser-calib
Flow:
    1. Do not open or use the lower camera.
    2. Do not use DEFAULT_THETA_HORIZONTAL / DEFAULT_THETA_VERTICAL.
    3. Read the current gimbal attitude as the current gimbal-camera center angle.
    4. Open the gimbal-camera view.
    5. Click the target in the gimbal-camera view, then press s.
    6. Convert the click offset to a gimbal control angle using FOV_X/FOV_Y.
    7. Move the gimbal so the camera center aligns with the clicked target.
    8. Start the laser grid scan and scan the full grid by default.
    9. Print LASER_COMP_AZ / LASER_COMP_EL for camera-center angle to laser angle.

Common scan options:
    --scan-az-start-offset 0.0
    --scan-az-end-offset 1.0
    --scan-el-start-offset 0.0
    --scan-el-end-offset -1.0
    --scan-step 0.3
    --scan-settle-timeout-seconds 3.0
    --scan-samples-per-point 5
    --scan-min-valid-samples 3
    --scan-pattern snake

Meaning of laser compensation:
    laser_ctrl_az = camera_ctrl_az + LASER_COMP_AZ
    laser_ctrl_el = camera_ctrl_el + LASER_COMP_EL

This script does not modify DEVICE_THETA and does not mix laser compensation
into lower-camera calibration parameters.
"""

import argparse
import csv
import json
import os
import threading
import time
from typing import List, Optional, Tuple

from calibration_matrix_to_gimbal import (
    FOV_X,
    FOV_Y,
    IMG_H,
    IMG_W,
    calibration_ui_to_ctrl_angles,
    compute_camera_angle_from_click,
    compute_gimbal_offset_from_click,
    draw_overlay,
    interactive_camera_click,
    open_camera_capture,
)
from main_tracking_v9 import GIMBAL_PORT, LASER_PORT, angular_diff
from sddm_laser import SDDMLaser
from validate_theta_to_gimbal import (
    _format_attitude,
    connect_gimbal,
    wait_gimbal_until_settled_without_view,
)


# ============================================================
# User-editable defaults.
# You can run this script directly after editing this block:
#     python validate_theta_to_gimbal_laser.py
# Command-line arguments still override these values.
# ============================================================
DEFAULT_THETA_HORIZONTAL = 60
DEFAULT_THETA_VERTICAL = 9.4
DEFAULT_LOWER_CAMERA = "000000003"
DEFAULT_GIMBAL_CAMERA = "000000008"
DEFAULT_GIMBAL_PORT = GIMBAL_PORT or os.getenv("GIMBAL_PORT") or "COM8"
DEFAULT_LASER_PORT = LASER_PORT or os.getenv("LASER_PORT") or "COM12"
DEFAULT_SETTLE_THRESHOLD = 0.2
DEFAULT_SETTLE_HOLD_SECONDS = 0.4
DEFAULT_SETTLE_TIMEOUT_SECONDS = 15.0
DEFAULT_SCAN_SETTLE_TIMEOUT_SECONDS = 3.0
DEFAULT_OVERLAY_SCALE = 1.5
DEFAULT_LASER_POLL_SECONDS = 0.05
DEFAULT_LASER_NO_VALID_PRINT_SECONDS = 1.0
DEFAULT_LASER_DEBUG = False

LASER_SCAN_CSV_FIELDS = [
    "timestamp",
    "center_ctrl_az",
    "center_ctrl_el",
    "scan_index",
    "az_offset",
    "el_offset",
    "scan_cmd_az",
    "scan_cmd_el",
    "actual_az",
    "actual_el",
    "settled",
    "settle_err_az",
    "settle_err_el",
    "samples_per_point",
    "valid_count",
    "invalid_count",
    "distance_median",
    "distance_min",
    "distance_max",
    "distance_mean",
    "raw_distances",
    "hit",
    "laser_comp_az",
    "laser_comp_el",
    "actual_laser_comp_az",
    "actual_laser_comp_el",
    "note",
]


class LaserDistanceWorker:
    def __init__(
        self,
        laser_port: str,
        poll_seconds: float,
        no_valid_print_seconds: float,
        debug: bool,
    ):
        self.laser_port = laser_port
        self.poll_seconds = max(0.0, float(poll_seconds))
        self.no_valid_print_seconds = max(0.1, float(no_valid_print_seconds))
        self.debug = debug
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.status = "idle"
        self.error = None
        self.latest_distance = None
        self.latest_distance_ts = None
        self.valid_count = 0
        self.invalid_count = 0

    def start(self) -> None:
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._run, name="laser_distance_worker", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "error": self.error,
                "latest_distance": self.latest_distance,
                "latest_distance_ts": self.latest_distance_ts,
                "valid_count": self.valid_count,
                "invalid_count": self.invalid_count,
            }

    def _set_status(self, status: str, error: Optional[str] = None) -> None:
        with self.lock:
            self.status = status
            self.error = error

    def _run(self) -> None:
        laser = None
        last_no_valid_print = 0.0
        try:
            print(f"[LaserValidate] opening laser on {self.laser_port}")
            self._set_status("opening")
            laser = SDDMLaser(self.laser_port)
            laser.start_measurement(continuous=True)
            self._set_status("running")
            print("[LaserValidate] continuous laser reading started; camera view stays open; press q/Esc to stop")

            while not self.stop_event.is_set():
                dist = laser.read_distance(debug=self.debug)
                now = time.monotonic()
                with self.lock:
                    if dist is None:
                        self.invalid_count += 1
                        invalid_count = self.invalid_count
                    else:
                        self.valid_count += 1
                        self.latest_distance = dist
                        self.latest_distance_ts = now
                        valid_count = self.valid_count

                if dist is None:
                    if now - last_no_valid_print >= self.no_valid_print_seconds:
                        print(f"[LaserValidate] no valid laser distance yet; invalid_reads={invalid_count}")
                        last_no_valid_print = now
                else:
                    print(f"[LaserValidate] distance_m={dist:.3f} valid_count={valid_count}")
                time.sleep(self.poll_seconds)
        except Exception as exc:
            msg = str(exc)
            self._set_status("error", msg)
            print(f"[LaserValidate][Error] laser worker failed: {msg}")
        finally:
            if laser is not None:
                laser.close()
            with self.lock:
                valid_count = self.valid_count
                invalid_count = self.invalid_count
                if self.status != "error":
                    self.status = "stopped"
            print(
                f"[LaserValidate] laser port released; "
                f"valid_reads={valid_count}, invalid_reads={invalid_count}"
            )


def show_laser_confirmation_view(
    camera_ref: str,
    target_ctrl_az: float,
    target_ctrl_el: float,
    final_attitude: Optional[Tuple[float, float, float]],
    final_err_az: float,
    final_err_el: float,
    settled: bool,
    width: int,
    height: int,
    overlay_scale: float,
    laser_port: str,
    laser_poll_seconds: float,
    laser_no_valid_print_seconds: float,
    laser_debug: bool,
) -> bool:
    cv2 = __import__("cv2")
    cap, camera_label = open_camera_capture(camera_ref, width, height)
    window_title = "gimbal laser confirmation view"
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    warned_size = False
    laser_worker = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError(f"gimbal camera frame read failed: {camera_ref} ({camera_label})")

            actual_h, actual_w = frame.shape[:2]
            if not warned_size and (actual_w != width or actual_h != height):
                print(f"[Warn] gimbal camera requested {width}x{height}, actual {actual_w}x{actual_h}")
                warned_size = True

            lines = [
                f"target_ctrl_az={target_ctrl_az:.4f} target_ctrl_el={target_ctrl_el:.4f}",
                _format_attitude(final_attitude),
                f"final_err_az={final_err_az:.4f} final_err_el={final_err_el:.4f}",
                "settled: YES" if settled else "settled: NO/TIMEOUT",
                "gimbal port: released",
            ]
            if laser_worker is None:
                lines.append("press l to start laser, q/Esc to skip")
            else:
                snap = laser_worker.snapshot()
                latest_distance = snap["latest_distance"]
                if latest_distance is None:
                    lines.append(f"laser: {snap['status']} no valid distance")
                else:
                    lines.append(f"laser: {snap['status']} distance_m={latest_distance:.3f}")
                lines.append(f"laser valid={snap['valid_count']} invalid={snap['invalid_count']}")
                if snap["error"]:
                    lines.append(f"laser error: {snap['error']}")
                lines.append("press q/Esc to stop laser and close view")
            draw_overlay(frame, None, lines, 1, window_title, overlay_scale)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("l") and laser_worker is None:
                print("[LaserValidate] laser measurement confirmed by user")
                laser_worker = LaserDistanceWorker(
                    laser_port=laser_port,
                    poll_seconds=laser_poll_seconds,
                    no_valid_print_seconds=laser_no_valid_print_seconds,
                    debug=laser_debug,
                )
                laser_worker.start()
            if key == ord("q") or key == 27:
                if laser_worker is None:
                    print("[LaserValidate] laser measurement skipped by user")
                    return False
                print("[LaserValidate] stopping laser measurement and closing view")
                return True
    finally:
        if laser_worker is not None:
            laser_worker.stop()
        cap.release()
        cv2.destroyWindow(window_title)


def _inclusive_float_range(start: float, end: float, step: float) -> List[float]:
    if step <= 0:
        raise ValueError("scan-step must be > 0")
    direction = 1.0 if end >= start else -1.0
    signed_step = abs(step) * direction
    values = []
    current = float(start)
    limit = float(end)
    eps = abs(step) * 1e-6

    if direction > 0:
        while current <= limit + eps:
            values.append(round(current, 10))
            current += signed_step
    else:
        while current >= limit - eps:
            values.append(round(current, 10))
            current += signed_step
    if not values or abs(values[-1] - limit) > eps:
        values.append(round(limit, 10))
    return values


def generate_scan_offsets(args) -> List[Tuple[float, float]]:
    az_offsets = _inclusive_float_range(
        args.scan_az_start_offset,
        args.scan_az_end_offset,
        args.scan_step,
    )
    el_offsets = _inclusive_float_range(
        args.scan_el_start_offset,
        args.scan_el_end_offset,
        args.scan_step,
    )

    offsets = []
    for row_idx, el_offset in enumerate(el_offsets):
        row_az_offsets = az_offsets
        if args.scan_pattern == "snake" and row_idx % 2 == 1:
            row_az_offsets = list(reversed(az_offsets))
        for az_offset in row_az_offsets:
            offsets.append((az_offset, el_offset))
    return offsets


def read_laser_samples(
    laser,
    samples_per_point: int,
    poll_seconds: float,
    distance_min: float,
    distance_max: float,
    debug: bool = False,
    view_callback=None,
):
    raw_distances = []
    valid_distances = []

    for sample_idx in range(1, max(0, int(samples_per_point)) + 1):
        dist = laser.read_distance(debug=debug)
        raw_distances.append(dist)
        if dist is not None and dist != -1 and distance_min <= dist <= distance_max:
            valid_distances.append(float(dist))
        if view_callback is not None:
            view_callback(sample_idx, dist, raw_distances, valid_distances)
        end_ts = time.monotonic() + max(0.0, float(poll_seconds))
        while time.monotonic() < end_ts:
            if view_callback is not None:
                view_callback(sample_idx, dist, raw_distances, valid_distances)
            time.sleep(0.02)

    valid_count = len(valid_distances)
    invalid_count = len(raw_distances) - valid_count
    sorted_valid = sorted(valid_distances)
    if valid_count == 0:
        distance_median = None
        distance_min_value = None
        distance_max_value = None
        distance_mean = None
    else:
        mid = valid_count // 2
        if valid_count % 2:
            distance_median = sorted_valid[mid]
        else:
            distance_median = (sorted_valid[mid - 1] + sorted_valid[mid]) / 2.0
        distance_min_value = sorted_valid[0]
        distance_max_value = sorted_valid[-1]
        distance_mean = sum(valid_distances) / valid_count

    return {
        "raw_distances": raw_distances,
        "valid_distances": valid_distances,
        "valid_count": valid_count,
        "invalid_count": invalid_count,
        "distance_median": distance_median,
        "distance_min": distance_min_value,
        "distance_max": distance_max_value,
        "distance_mean": distance_mean,
    }


def _csv_value(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6f}"
    return value


def append_laser_scan_csv(path: str, row: dict) -> None:
    exists = os.path.exists(path)
    if exists and os.path.getsize(path) > 0:
        with open(path, "rb") as f:
            f.seek(-1, os.SEEK_END)
            if f.read(1) != b"\n":
                with open(path, "ab") as append_f:
                    append_f.write(b"\n")

    write_header = not exists or os.path.getsize(path) == 0
    normalized_row = {field: _csv_value(row.get(field, "")) for field in LASER_SCAN_CSV_FIELDS}
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=LASER_SCAN_CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(normalized_row)


def _distance_spread(hit: dict) -> float:
    distance_min_value = hit.get("distance_min")
    distance_max_value = hit.get("distance_max")
    if distance_min_value is None or distance_max_value is None:
        return float("inf")
    return float(distance_max_value) - float(distance_min_value)


def _draw_laser_scan_view(
    cap,
    window_title: str,
    overlay_scale: float,
    idx: int,
    total: int,
    phase: str,
    center_ctrl_az: float,
    center_ctrl_el: float,
    scan_cmd_az: float,
    scan_cmd_el: float,
    az_offset: float,
    el_offset: float,
    final_attitude: Optional[Tuple[float, float, float]],
    err_az: float,
    err_el: float,
    settled: bool,
    valid_count: int,
    invalid_count: int,
    latest_distance,
) -> None:
    cv2 = __import__("cv2")
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("gimbal camera frame read failed during laser scan")

    distance_text = "None" if latest_distance is None else f"{float(latest_distance):.3f}"
    lines = [
        f"scan {idx}/{total} phase={phase}",
        f"center_az={center_ctrl_az:.4f} center_el={center_ctrl_el:.4f}",
        f"scan_az={scan_cmd_az:.4f} scan_el={scan_cmd_el:.4f}",
        f"offset_az={az_offset:.4f} offset_el={el_offset:.4f}",
        _format_attitude(final_attitude),
        f"err_az={err_az:.4f} err_el={err_el:.4f} settled={settled}",
        f"laser valid={valid_count} invalid={invalid_count} latest={distance_text}",
        "press q/Esc to abort scan",
    ]
    draw_overlay(frame, None, lines, 1, window_title, overlay_scale)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or key == 27:
        raise KeyboardInterrupt("laser scan aborted by user")


def wait_gimbal_until_settled_with_scan_view(
    gimbal,
    cap,
    window_title: str,
    overlay_scale: float,
    idx: int,
    total: int,
    center_ctrl_az: float,
    center_ctrl_el: float,
    scan_cmd_az: float,
    scan_cmd_el: float,
    az_offset: float,
    el_offset: float,
    settle_threshold: float,
    settle_hold_seconds: float,
    settle_timeout_seconds: float,
) -> Tuple[bool, Optional[Tuple[float, float, float]], float, float]:
    start_ts = time.monotonic()
    settled_since = None
    last_attitude = None
    last_err_az = float("inf")
    last_err_el = float("inf")

    while True:
        now = time.monotonic()
        last_attitude = gimbal.get_attitude()
        if last_attitude is not None:
            curr_el, curr_az, _ = last_attitude
            last_err_az = angular_diff(scan_cmd_az, curr_az)
            last_err_el = scan_cmd_el - curr_el
            if abs(last_err_az) <= settle_threshold and abs(last_err_el) <= settle_threshold:
                if settled_since is None:
                    settled_since = now
            else:
                settled_since = None

        hold = 0.0 if settled_since is None else now - settled_since
        elapsed = now - start_ts
        settled = settled_since is not None and hold >= settle_hold_seconds
        _draw_laser_scan_view(
            cap=cap,
            window_title=window_title,
            overlay_scale=overlay_scale,
            idx=idx,
            total=total,
            phase=f"settling {elapsed:.1f}s hold={hold:.1f}s",
            center_ctrl_az=center_ctrl_az,
            center_ctrl_el=center_ctrl_el,
            scan_cmd_az=scan_cmd_az,
            scan_cmd_el=scan_cmd_el,
            az_offset=az_offset,
            el_offset=el_offset,
            final_attitude=last_attitude,
            err_az=last_err_az,
            err_el=last_err_el,
            settled=settled,
            valid_count=0,
            invalid_count=0,
            latest_distance=None,
        )

        if settled:
            return True, last_attitude, last_err_az, last_err_el
        if elapsed >= settle_timeout_seconds:
            return False, last_attitude, last_err_az, last_err_el
        time.sleep(0.02)


def sleep_with_scan_view(
    seconds: float,
    cap,
    window_title: str,
    overlay_scale: float,
    idx: int,
    total: int,
    center_ctrl_az: float,
    center_ctrl_el: float,
    scan_cmd_az: float,
    scan_cmd_el: float,
    az_offset: float,
    el_offset: float,
    final_attitude: Optional[Tuple[float, float, float]],
    err_az: float,
    err_el: float,
    settled: bool,
) -> None:
    end_ts = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < end_ts:
        _draw_laser_scan_view(
            cap=cap,
            window_title=window_title,
            overlay_scale=overlay_scale,
            idx=idx,
            total=total,
            phase="post-settle dwell",
            center_ctrl_az=center_ctrl_az,
            center_ctrl_el=center_ctrl_el,
            scan_cmd_az=scan_cmd_az,
            scan_cmd_el=scan_cmd_el,
            az_offset=az_offset,
            el_offset=el_offset,
            final_attitude=final_attitude,
            err_az=err_az,
            err_el=err_el,
            settled=settled,
            valid_count=0,
            invalid_count=0,
            latest_distance=None,
        )
        time.sleep(0.02)


def run_laser_scan(args, center_ctrl_az: float, center_ctrl_el: float) -> None:
    offsets = generate_scan_offsets(args)
    print(
        f"[LaserScan] start: points={len(offsets)} center_az={center_ctrl_az:.6f} "
        f"center_el={center_ctrl_el:.6f} pattern={args.scan_pattern} output={args.scan_output_csv}"
    )

    gimbal = None
    laser = None
    cap = None
    window_title = "laser scan gimbal live view"
    hits = []

    try:
        cv2 = __import__("cv2")
        cap, camera_label = open_camera_capture(args.gimbal_camera, args.gimbal_camera_w, args.gimbal_camera_h)
        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
        print(f"[LaserScan] gimbal camera opened: {camera_label}")
        gimbal = connect_gimbal(args.gimbal_port)
        laser = SDDMLaser(args.laser_port)
        laser.start_measurement(continuous=True)

        for idx, (az_offset, el_offset) in enumerate(offsets, start=1):
            scan_cmd_az = center_ctrl_az + az_offset
            scan_cmd_el = center_ctrl_el + el_offset
            gimbal.set_attitude(elevation=scan_cmd_el, azimuth=scan_cmd_az)
            settled, final_attitude, final_err_az, final_err_el = wait_gimbal_until_settled_with_scan_view(
                gimbal=gimbal,
                cap=cap,
                window_title=window_title,
                overlay_scale=args.overlay_scale,
                idx=idx,
                total=len(offsets),
                center_ctrl_az=center_ctrl_az,
                center_ctrl_el=center_ctrl_el,
                scan_cmd_az=scan_cmd_az,
                scan_cmd_el=scan_cmd_el,
                az_offset=az_offset,
                el_offset=el_offset,
                settle_threshold=args.settle_threshold,
                settle_hold_seconds=args.settle_hold_seconds,
                settle_timeout_seconds=args.scan_settle_timeout_seconds,
            )
            sleep_with_scan_view(
                seconds=args.scan_settle_seconds,
                cap=cap,
                window_title=window_title,
                overlay_scale=args.overlay_scale,
                idx=idx,
                total=len(offsets),
                center_ctrl_az=center_ctrl_az,
                center_ctrl_el=center_ctrl_el,
                scan_cmd_az=scan_cmd_az,
                scan_cmd_el=scan_cmd_el,
                az_offset=az_offset,
                el_offset=el_offset,
                final_attitude=final_attitude,
                err_az=final_err_az,
                err_el=final_err_el,
                settled=settled,
            )

            actual_az = None
            actual_el = None
            if final_attitude is not None:
                actual_el, actual_az, _ = final_attitude

            sample_stats = read_laser_samples(
                laser=laser,
                samples_per_point=args.scan_samples_per_point,
                poll_seconds=args.laser_poll_seconds,
                distance_min=args.scan_distance_min,
                distance_max=args.scan_distance_max,
                debug=args.laser_debug,
                view_callback=lambda sample_idx, dist, raw, valid: _draw_laser_scan_view(
                    cap=cap,
                    window_title=window_title,
                    overlay_scale=args.overlay_scale,
                    idx=idx,
                    total=len(offsets),
                    phase=f"sampling {sample_idx}/{args.scan_samples_per_point}",
                    center_ctrl_az=center_ctrl_az,
                    center_ctrl_el=center_ctrl_el,
                    scan_cmd_az=scan_cmd_az,
                    scan_cmd_el=scan_cmd_el,
                    az_offset=az_offset,
                    el_offset=el_offset,
                    final_attitude=final_attitude,
                    err_az=final_err_az,
                    err_el=final_err_el,
                    settled=settled,
                    valid_count=len(valid),
                    invalid_count=len(raw) - len(valid),
                    latest_distance=dist,
                ),
            )
            valid_count = sample_stats["valid_count"]
            hit = valid_count >= args.scan_min_valid_samples
            laser_comp_az = angular_diff(scan_cmd_az, center_ctrl_az)
            laser_comp_el = scan_cmd_el - center_ctrl_el
            actual_laser_comp_az = angular_diff(actual_az, center_ctrl_az) if actual_az is not None else None
            actual_laser_comp_el = actual_el - center_ctrl_el if actual_el is not None else None
            note_parts = []
            if not settled:
                note_parts.append("settle_timeout")
            if valid_count == 0:
                note_parts.append("no_valid_laser")
            if hit:
                note_parts.append("hit")

            row = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "center_ctrl_az": center_ctrl_az,
                "center_ctrl_el": center_ctrl_el,
                "scan_index": idx,
                "az_offset": az_offset,
                "el_offset": el_offset,
                "scan_cmd_az": scan_cmd_az,
                "scan_cmd_el": scan_cmd_el,
                "actual_az": actual_az,
                "actual_el": actual_el,
                "settled": int(bool(settled)),
                "settle_err_az": final_err_az,
                "settle_err_el": final_err_el,
                "samples_per_point": args.scan_samples_per_point,
                "valid_count": valid_count,
                "invalid_count": sample_stats["invalid_count"],
                "distance_median": sample_stats["distance_median"],
                "distance_min": sample_stats["distance_min"],
                "distance_max": sample_stats["distance_max"],
                "distance_mean": sample_stats["distance_mean"],
                "raw_distances": json.dumps(sample_stats["raw_distances"], ensure_ascii=False),
                "hit": int(bool(hit)),
                "laser_comp_az": laser_comp_az,
                "laser_comp_el": laser_comp_el,
                "actual_laser_comp_az": actual_laser_comp_az,
                "actual_laser_comp_el": actual_laser_comp_el,
                "note": ";".join(note_parts),
            }
            append_laser_scan_csv(args.scan_output_csv, row)

            median_text = "None" if sample_stats["distance_median"] is None else f"{sample_stats['distance_median']:.3f}"
            print(
                f"[LaserScan] idx={idx} az={scan_cmd_az:.6f} el={scan_cmd_el:.6f} "
                f"offset=({az_offset:.6f},{el_offset:.6f}) "
                f"valid={valid_count}/{args.scan_samples_per_point} median={median_text} hit={hit}"
            )
            if hit:
                hits.append(row)
                print(
                    f"[LaserScan][Hit] scan_cmd_az={scan_cmd_az:.6f} scan_cmd_el={scan_cmd_el:.6f} "
                    f"laser_comp_az={laser_comp_az:.6f} laser_comp_el={laser_comp_el:.6f} "
                    f"distance_median={median_text}"
                )
                if args.scan_stop_on_first_hit:
                    print("[LaserScan] stop on first hit enabled; stopping scan")
                    break

        if not hits:
            print("[LaserScan][Warn] no valid laser hit found. Try larger range or opposite scan direction.")
            return

        best = sorted(
            hits,
            key=lambda item: (
                -int(item["valid_count"]),
                _distance_spread(item),
                float("inf") if item["distance_median"] is None else float(item["distance_median"]),
            ),
        )[0]
        best_scan_az = float(best["scan_cmd_az"])
        best_scan_el = float(best["scan_cmd_el"])
        laser_comp_az = angular_diff(best_scan_az, center_ctrl_az)
        laser_comp_el = best_scan_el - center_ctrl_el

        print("[LaserScan][Best]")
        print(f"center_ctrl_az={center_ctrl_az:.6f}")
        print(f"center_ctrl_el={center_ctrl_el:.6f}")
        print(f"best_scan_az={best_scan_az:.6f}")
        print(f"best_scan_el={best_scan_el:.6f}")
        print(f"LASER_COMP_AZ={laser_comp_az:.6f}")
        print(f"LASER_COMP_EL={laser_comp_el:.6f}")
        print(f"distance_median={float(best['distance_median']):.6f}")
        print(f"valid_count={int(best['valid_count'])}")
    except KeyboardInterrupt as exc:
        print(f"\n[LaserScan] aborted: {exc}")
    finally:
        if cap is not None:
            cap.release()
            cv2 = __import__("cv2")
            cv2.destroyWindow(window_title)
            print("[LaserScan] gimbal camera released")
        if laser is not None:
            laser.close()
            print("[LaserScan] laser port released")
        if gimbal is not None:
            driver = getattr(gimbal, "driver", None)
            is_connected = getattr(driver, "is_connected", None)
            if callable(is_connected) and is_connected():
                gimbal.close()
                print("[LaserScan] gimbal port released")


def run_gimbal_camera_laser_calibration(args) -> int:
    gimbal = connect_gimbal(args.gimbal_port)
    try:
        attitude = gimbal.get_attitude()
    finally:
        driver = getattr(gimbal, "driver", None)
        is_connected = getattr(driver, "is_connected", None)
        if callable(is_connected) and is_connected():
            gimbal.close()
            print("[GimbalLaserCalib] gimbal port released before camera click")

    if attitude is None:
        raise RuntimeError("failed to read current gimbal attitude before gimbal-camera click")

    current_el, current_az, _ = attitude
    center_before_click_az = float(current_az)
    center_before_click_el = float(current_el)
    print(
        f"[GimbalLaserCalib] current camera center: "
        f"az={center_before_click_az:.6f}, el={center_before_click_el:.6f}"
    )

    def gimbal_lines(frame, click):
        h, w = frame.shape[:2]
        lines = [
            "mode=gimbal-camera laser calibration",
            f"current_center_az={center_before_click_az:.6f} current_center_el={center_before_click_el:.6f}",
            f"image={w}x{h} fov={FOV_X:.6f}x{FOV_Y:.6f}",
        ]
        if click is not None:
            offset = compute_gimbal_offset_from_click(click[0], click[1], w, h, FOV_X, FOV_Y)
            target_ctrl_az = (center_before_click_az + offset["offset_az"]) % 360.0
            target_ctrl_el = center_before_click_el + offset["offset_el"]
            lines.extend([
                f"dx={offset['dx']:.2f} dy={offset['dy']:.2f}",
                f"offset_az={offset['offset_az']:.6f} offset_el={offset['offset_el']:.6f}",
                f"target_ctrl_az={target_ctrl_az:.6f} target_ctrl_el={target_ctrl_el:.6f}",
            ])
        return lines

    gimbal_frame, click_x, click_y, _valid, gimbal_source = interactive_camera_click(
        args.gimbal_camera,
        args.gimbal_camera_w,
        args.gimbal_camera_h,
        "gimbal camera target selection",
        gimbal_lines,
        1,
        args.overlay_scale,
    )
    image_h, image_w = gimbal_frame.shape[:2]
    offset = compute_gimbal_offset_from_click(click_x, click_y, image_w, image_h, FOV_X, FOV_Y)
    ctrl_az = (center_before_click_az + offset["offset_az"]) % 360.0
    ctrl_el = center_before_click_el + offset["offset_el"]

    print(f"[GimbalLaserCalib] gimbal_source={gimbal_source}")
    print(
        f"[GimbalLaserCalib] click=({click_x:.1f}, {click_y:.1f}), "
        f"offset_az={offset['offset_az']:.6f}, offset_el={offset['offset_el']:.6f}"
    )
    print(f"[GimbalLaserCalib] target camera center ctrl_az={ctrl_az:.6f}, ctrl_el={ctrl_el:.6f}")

    gimbal = connect_gimbal(args.gimbal_port)
    try:
        before_attitude = gimbal.get_attitude()
        print(f"[GimbalLaserCalib] before command: {_format_attitude(before_attitude)}")
        print(f"[GimbalLaserCalib] sending gimbal command: az={ctrl_az:.6f}, el={ctrl_el:.6f}")
        gimbal.set_attitude(elevation=ctrl_el, azimuth=ctrl_az)
        settled, _final_attitude, final_err_az, final_err_el = wait_gimbal_until_settled_without_view(
            gimbal=gimbal,
            target_ctrl_az=ctrl_az,
            target_ctrl_el=ctrl_el,
            settle_threshold=args.settle_threshold,
            settle_hold_seconds=args.settle_hold_seconds,
            settle_timeout_seconds=args.settle_timeout_seconds,
        )
        print(
            f"[GimbalLaserCalib] camera-center target settled={settled} "
            f"err_az={final_err_az:.6f}, err_el={final_err_el:.6f}"
        )
    finally:
        driver = getattr(gimbal, "driver", None)
        is_connected = getattr(driver, "is_connected", None)
        if callable(is_connected) and is_connected():
            gimbal.close()
            print("[GimbalLaserCalib] gimbal port released before laser scan")

    run_laser_scan(args, center_ctrl_az=ctrl_az, center_ctrl_el=ctrl_el)
    return 0 if settled else 2


def run(args) -> int:
    if args.gimbal_camera_laser_calib:
        return run_gimbal_camera_laser_calibration(args)

    theta_cfg = {
        "theta_horizontal": float(args.theta_horizontal),
        "theta_vertical": float(args.theta_vertical),
    }

    def lower_lines(frame, click):
        h, w = frame.shape[:2]
        lines = [
            f"theta_h={theta_cfg['theta_horizontal']:.6f} theta_v={theta_cfg['theta_vertical']:.6f}",
            f"image={w}x{h} fov={FOV_X:.6f}x{FOV_Y:.6f}",
        ]
        if click is not None:
            result = compute_camera_angle_from_click(click[0], click[1], w, h, theta_cfg)
            ctrl_az, ctrl_el = calibration_ui_to_ctrl_angles(result["calc_ui_az"], result["calc_ui_el"])
            lines.extend([
                f"dx={result['dx']:.2f} dy={result['dy']:.2f}",
                f"offset_az={result['offset_az']:.6f} offset_el={result['offset_el']:.6f}",
                f"target_az={result['calc_ui_az']:.6f} target_el={result['calc_ui_el']:.6f}",
                f"ctrl_az={ctrl_az:.6f} ctrl_el={ctrl_el:.6f}",
            ])
        return lines

    lower_frame, lower_x, lower_y, _valid, lower_source = interactive_camera_click(
        args.lower_camera,
        args.lower_camera_w,
        args.lower_camera_h,
        "lower camera target selection",
        lower_lines,
        1,
        args.overlay_scale,
    )
    lower_h, lower_w = lower_frame.shape[:2]
    lower_result = compute_camera_angle_from_click(lower_x, lower_y, lower_w, lower_h, theta_cfg)
    ctrl_az, ctrl_el = calibration_ui_to_ctrl_angles(lower_result["calc_ui_az"], lower_result["calc_ui_el"])

    print(f"[LaserValidate] lower_source={lower_source}")
    print(
        f"[LaserValidate] lower click=({lower_x:.1f}, {lower_y:.1f}), "
        f"offset_az={lower_result['offset_az']:.6f}, offset_el={lower_result['offset_el']:.6f}"
    )
    print(
        f"[LaserValidate] target_az={lower_result['calc_ui_az']:.6f}, "
        f"target_el={lower_result['calc_ui_el']:.6f}, "
        f"ctrl_az={ctrl_az:.6f}, ctrl_el={ctrl_el:.6f}"
    )

    gimbal = connect_gimbal(args.gimbal_port)
    try:
        before_attitude = gimbal.get_attitude()
        print(f"[LaserValidate] before command: {_format_attitude(before_attitude)}")
        if before_attitude is not None:
            curr_el, curr_az, _ = before_attitude
            print(
                f"[LaserValidate] expected move: "
                f"d_az={angular_diff(ctrl_az, curr_az):.6f}, "
                f"d_el={ctrl_el - curr_el:.6f}"
            )
            if abs(angular_diff(ctrl_az, curr_az)) <= args.settle_threshold and abs(ctrl_el - curr_el) <= args.settle_threshold:
                print(
                    "[LaserValidate][Warn] target is already within settle threshold; "
                    "the gimbal may not visibly move."
                )

        print(f"[LaserValidate] sending gimbal command: az={ctrl_az:.6f}, el={ctrl_el:.6f}")
        gimbal.set_attitude(elevation=ctrl_el, azimuth=ctrl_az)
        settled, final_attitude, final_err_az, final_err_el = wait_gimbal_until_settled_without_view(
            gimbal=gimbal,
            target_ctrl_az=ctrl_az,
            target_ctrl_el=ctrl_el,
            settle_threshold=args.settle_threshold,
            settle_hold_seconds=args.settle_hold_seconds,
            settle_timeout_seconds=args.settle_timeout_seconds,
        )
    finally:
        driver = getattr(gimbal, "driver", None)
        is_connected = getattr(driver, "is_connected", None)
        if callable(is_connected) and is_connected():
            gimbal.close()
            print("[LaserValidate] gimbal port released")

    if args.enable_laser_scan:
        run_laser_scan(args, center_ctrl_az=ctrl_az, center_ctrl_el=ctrl_el)
    else:
        show_laser_confirmation_view(
            camera_ref=args.gimbal_camera,
            target_ctrl_az=ctrl_az,
            target_ctrl_el=ctrl_el,
            final_attitude=final_attitude,
            final_err_az=final_err_az,
            final_err_el=final_err_el,
            settled=settled,
            width=args.gimbal_camera_w,
            height=args.gimbal_camera_h,
            overlay_scale=args.overlay_scale,
            laser_port=args.laser_port,
            laser_poll_seconds=args.laser_poll_seconds,
            laser_no_valid_print_seconds=args.laser_no_valid_print_seconds,
            laser_debug=args.laser_debug,
        )
    return 0 if settled else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a calibrated theta, settle the gimbal, then optionally start "
            "continuous SDDM laser distance reading."
        )
    )
    parser.add_argument("--theta-horizontal", type=float, default=DEFAULT_THETA_HORIZONTAL, help="Suggested calibration-space theta_horizontal.")
    parser.add_argument("--theta-vertical", type=float, default=DEFAULT_THETA_VERTICAL, help="Suggested calibration-space theta_vertical.")
    parser.add_argument("--lower-camera", default=DEFAULT_LOWER_CAMERA, help="DirectShow camera name or OpenCV index, e.g. 000000002 or dshow:2.")
    parser.add_argument("--gimbal-camera", default=DEFAULT_GIMBAL_CAMERA, help="DirectShow camera name or OpenCV index, e.g. 000000008 or dshow:1.")
    parser.add_argument("--lower-camera-w", type=int, default=int(IMG_W))
    parser.add_argument("--lower-camera-h", type=int, default=int(IMG_H))
    parser.add_argument("--gimbal-camera-w", type=int, default=int(IMG_W))
    parser.add_argument("--gimbal-camera-h", type=int, default=int(IMG_H))
    parser.add_argument("--gimbal-port", default=DEFAULT_GIMBAL_PORT)
    parser.add_argument("--laser-port", default=DEFAULT_LASER_PORT)
    parser.add_argument("--settle-threshold", type=float, default=DEFAULT_SETTLE_THRESHOLD)
    parser.add_argument("--settle-hold-seconds", type=float, default=DEFAULT_SETTLE_HOLD_SECONDS)
    parser.add_argument("--settle-timeout-seconds", type=float, default=DEFAULT_SETTLE_TIMEOUT_SECONDS)
    parser.add_argument("--overlay-scale", type=float, default=DEFAULT_OVERLAY_SCALE)
    parser.add_argument("--laser-poll-seconds", type=float, default=DEFAULT_LASER_POLL_SECONDS)
    parser.add_argument("--laser-no-valid-print-seconds", type=float, default=DEFAULT_LASER_NO_VALID_PRINT_SECONDS)
    parser.add_argument("--laser-debug", action="store_true", default=DEFAULT_LASER_DEBUG)
    parser.add_argument("--gimbal-camera-laser-calib", action="store_true", default=False)
    parser.add_argument("--enable-laser-scan", action="store_true", default=False)
    parser.add_argument("--scan-az-start-offset", type=float, default=0.0)
    parser.add_argument("--scan-az-end-offset", type=float, default=1.0)
    parser.add_argument("--scan-el-start-offset", type=float, default=0.0)
    parser.add_argument("--scan-el-end-offset", type=float, default=-1.0)
    parser.add_argument("--scan-step", type=float, default=0.3)
    parser.add_argument("--scan-settle-timeout-seconds", type=float, default=DEFAULT_SCAN_SETTLE_TIMEOUT_SECONDS)
    parser.add_argument("--scan-settle-seconds", type=float, default=0.3)
    parser.add_argument("--scan-samples-per-point", type=int, default=5)
    parser.add_argument("--scan-min-valid-samples", type=int, default=3)
    parser.add_argument("--scan-distance-min", type=float, default=0.0)
    parser.add_argument("--scan-distance-max", type=float, default=2000.0)
    parser.add_argument("--scan-output-csv", default="laser_scan_calibration.csv")
    parser.add_argument("--scan-stop-on-first-hit", action="store_true", default=False)
    parser.add_argument("--scan-pattern", choices=("grid", "snake"), default="snake")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(f"[LaserValidate][Error] {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
