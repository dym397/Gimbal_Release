import argparse
import math
import os
import time
from typing import Optional, Tuple

from calibration_matrix_to_gimbal import (
    FOV_X,
    FOV_Y,
    IMG_H,
    IMG_W,
    calibration_ui_to_ctrl_angles,
    compute_camera_angle_from_click,
    draw_overlay,
    interactive_camera_click,
    open_camera_capture,
)
from gimbal_interface import GT06ZAdapter
from main_tracking_v9 import GIMBAL_PORT, angular_diff


# ============================================================
# User-editable defaults.
# You can run this script directly after editing this block:
#     python validate_theta_to_gimbal.py
# Command-line arguments still override these values.
# ============================================================
DEFAULT_THETA_HORIZONTAL = 42.6
DEFAULT_THETA_VERTICAL = 9.4
DEFAULT_LOWER_CAMERA = "000000003"
DEFAULT_GIMBAL_CAMERA = "000000008"
DEFAULT_GIMBAL_PORT = GIMBAL_PORT or os.getenv("GIMBAL_PORT") or "COM8"
DEFAULT_SETTLE_THRESHOLD = 0.2
DEFAULT_SETTLE_HOLD_SECONDS = 0.4
DEFAULT_SETTLE_TIMEOUT_SECONDS = 15.0
DEFAULT_OVERLAY_SCALE = 1.5
DEFAULT_KEEP_VIEW_AFTER_SETTLE = True
DEFAULT_RELEASE_GIMBAL_BEFORE_VIEW = True


def connect_gimbal(port: str) -> GT06ZAdapter:
    gimbal = GT06ZAdapter(port)
    if not gimbal.connect():
        raise RuntimeError(f"failed to connect gimbal on {port}")
    if not gimbal.wait_ready():
        gimbal.close()
        raise RuntimeError(f"gimbal is not ready on {port}")
    return gimbal


def _format_attitude(attitude: Optional[Tuple[float, float, float]]) -> str:
    if attitude is None:
        return "current=unreadable"
    el, az, _ = attitude
    return f"current_az={az:.4f} current_el={el:.4f}"


def show_gimbal_until_settled(
    gimbal: GT06ZAdapter,
    camera_ref: str,
    target_ctrl_az: float,
    target_ctrl_el: float,
    width: int,
    height: int,
    settle_threshold: float,
    settle_hold_seconds: float,
    settle_timeout_seconds: float,
    overlay_scale: float,
    keep_view_after_settle: bool,
) -> bool:
    cv2 = __import__("cv2")
    cap, camera_label = open_camera_capture(camera_ref, width, height)
    window_title = "gimbal validation live view"
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)

    start_ts = time.monotonic()
    settled_since = None
    last_query_ts = 0.0
    last_attitude = None
    last_err_az = math.inf
    last_err_el = math.inf
    warned_size = False
    gimbal_port_released = False

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError(f"gimbal camera frame read failed: {camera_ref} ({camera_label})")

            actual_h, actual_w = frame.shape[:2]
            if not warned_size and (actual_w != width or actual_h != height):
                print(f"[Warn] gimbal camera requested {width}x{height}, actual {actual_w}x{actual_h}")
                warned_size = True

            now = time.monotonic()
            if (now - last_query_ts) >= 0.12:
                last_attitude = gimbal.get_attitude()
                last_query_ts = now
                if last_attitude is not None:
                    curr_el, curr_az, _ = last_attitude
                    last_err_az = angular_diff(target_ctrl_az, curr_az)
                    last_err_el = target_ctrl_el - curr_el
                    if abs(last_err_az) <= settle_threshold and abs(last_err_el) <= settle_threshold:
                        if settled_since is None:
                            settled_since = now
                    else:
                        settled_since = None

            elapsed = now - start_ts
            is_settled = settled_since is not None and (now - settled_since) >= settle_hold_seconds
            is_timeout = elapsed >= settle_timeout_seconds
            lines = [
                f"target_ctrl_az={target_ctrl_az:.4f} target_ctrl_el={target_ctrl_el:.4f}",
                _format_attitude(last_attitude),
                f"err_az={last_err_az:.4f} err_el={last_err_el:.4f} threshold={settle_threshold:.3f}",
                f"elapsed={elapsed:.1f}s hold={0.0 if settled_since is None else now - settled_since:.1f}s",
                "settled: YES" if is_settled else "settled: NO",
                "q quit",
            ]
            if is_settled and keep_view_after_settle:
                lines.append("settled: keeping live view open; press q/Esc when done")
            draw_overlay(frame, None, lines, 1, window_title, overlay_scale)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                print("[Validate] user stopped live view")
                return bool(is_settled)
            if is_settled:
                print(
                    f"[Validate] settled: err_az={last_err_az:.4f}, "
                    f"err_el={last_err_el:.4f}, hold={settle_hold_seconds:.2f}s"
                )
                if not gimbal_port_released:
                    gimbal.close()
                    gimbal_port_released = True
                    print("[Validate] gimbal port released after settle")
                if not keep_view_after_settle:
                    return True
                final_attitude_text = _format_attitude(last_attitude)
                final_err_az = last_err_az
                final_err_el = last_err_el
                while True:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError(f"gimbal camera frame read failed: {camera_ref} ({camera_label})")
                    lines = [
                        f"target_ctrl_az={target_ctrl_az:.4f} target_ctrl_el={target_ctrl_el:.4f}",
                        final_attitude_text,
                        f"final_err_az={final_err_az:.4f} final_err_el={final_err_el:.4f}",
                        "settled: YES",
                        "gimbal port: released",
                        "inspection mode: press q/Esc when done",
                    ]
                    draw_overlay(frame, None, lines, 1, window_title, overlay_scale)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        return True
            if is_timeout:
                print(
                    f"[Validate][Warn] settle timeout: err_az={last_err_az:.4f}, "
                    f"err_el={last_err_el:.4f}, elapsed={elapsed:.1f}s"
                )
                return False
    finally:
        cap.release()
        cv2.destroyWindow(window_title)


def wait_gimbal_until_settled_without_view(
    gimbal: GT06ZAdapter,
    target_ctrl_az: float,
    target_ctrl_el: float,
    settle_threshold: float,
    settle_hold_seconds: float,
    settle_timeout_seconds: float,
) -> Tuple[bool, Optional[Tuple[float, float, float]], float, float]:
    start_ts = time.monotonic()
    settled_since = None
    last_attitude = None
    last_err_az = math.inf
    last_err_el = math.inf

    while True:
        now = time.monotonic()
        last_attitude = gimbal.get_attitude()
        if last_attitude is not None:
            curr_el, curr_az, _ = last_attitude
            last_err_az = angular_diff(target_ctrl_az, curr_az)
            last_err_el = target_ctrl_el - curr_el
            if abs(last_err_az) <= settle_threshold and abs(last_err_el) <= settle_threshold:
                if settled_since is None:
                    settled_since = now
            else:
                settled_since = None

        hold = 0.0 if settled_since is None else now - settled_since
        elapsed = now - start_ts
        print(
            f"\r[Validate] settling without camera view: {_format_attitude(last_attitude)} "
            f"err_az={last_err_az:.4f} err_el={last_err_el:.4f} hold={hold:.1f}s elapsed={elapsed:.1f}s",
            end="",
            flush=True,
        )

        if settled_since is not None and hold >= settle_hold_seconds:
            print()
            print(f"[Validate] settled before opening gimbal camera: err_az={last_err_az:.4f}, err_el={last_err_el:.4f}")
            return True, last_attitude, last_err_az, last_err_el
        if elapsed >= settle_timeout_seconds:
            print()
            print(f"[Validate][Warn] settle timeout before camera view: err_az={last_err_az:.4f}, err_el={last_err_el:.4f}")
            return False, last_attitude, last_err_az, last_err_el
        time.sleep(0.12)


def show_gimbal_inspection_view(
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
) -> None:
    cv2 = __import__("cv2")
    cap, camera_label = open_camera_capture(camera_ref, width, height)
    window_title = "gimbal validation live view"
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    warned_size = False

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
                "gimbal port: released before opening this view",
                "inspection mode: press q/Esc when done",
            ]
            draw_overlay(frame, None, lines, 1, window_title, overlay_scale)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                return
    finally:
        cap.release()
        cv2.destroyWindow(window_title)


def run(args) -> int:
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

    print(f"[Validate] lower_source={lower_source}")
    print(
        f"[Validate] lower click=({lower_x:.1f}, {lower_y:.1f}), "
        f"offset_az={lower_result['offset_az']:.6f}, offset_el={lower_result['offset_el']:.6f}"
    )
    print(
        f"[Validate] target_az={lower_result['calc_ui_az']:.6f}, "
        f"target_el={lower_result['calc_ui_el']:.6f}, "
        f"ctrl_az={ctrl_az:.6f}, ctrl_el={ctrl_el:.6f}"
    )

    gimbal = connect_gimbal(args.gimbal_port)
    try:
        before_attitude = gimbal.get_attitude()
        print(f"[Validate] before command: {_format_attitude(before_attitude)}")
        if before_attitude is not None:
            curr_el, curr_az, _ = before_attitude
            print(
                f"[Validate] expected move: "
                f"d_az={angular_diff(ctrl_az, curr_az):.6f}, "
                f"d_el={ctrl_el - curr_el:.6f}"
            )
            if abs(angular_diff(ctrl_az, curr_az)) <= args.settle_threshold and abs(ctrl_el - curr_el) <= args.settle_threshold:
                print(
                    "[Validate][Warn] target is already within settle threshold; "
                    "the gimbal may not visibly move."
                )
        print(f"[Validate] sending gimbal command: az={ctrl_az:.6f}, el={ctrl_el:.6f}")
        gimbal.set_attitude(elevation=ctrl_el, azimuth=ctrl_az)
        if args.release_gimbal_before_view:
            settled, final_attitude, final_err_az, final_err_el = wait_gimbal_until_settled_without_view(
                gimbal=gimbal,
                target_ctrl_az=ctrl_az,
                target_ctrl_el=ctrl_el,
                settle_threshold=args.settle_threshold,
                settle_hold_seconds=args.settle_hold_seconds,
                settle_timeout_seconds=args.settle_timeout_seconds,
            )
            gimbal.close()
            print("[Validate] gimbal port released before opening gimbal camera")
            show_gimbal_inspection_view(
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
            )
        else:
            settled = show_gimbal_until_settled(
                gimbal=gimbal,
                camera_ref=args.gimbal_camera,
                target_ctrl_az=ctrl_az,
                target_ctrl_el=ctrl_el,
                width=args.gimbal_camera_w,
                height=args.gimbal_camera_h,
                settle_threshold=args.settle_threshold,
                settle_hold_seconds=args.settle_hold_seconds,
                settle_timeout_seconds=args.settle_timeout_seconds,
                overlay_scale=args.overlay_scale,
                keep_view_after_settle=args.keep_view_after_settle,
            )
        return 0 if settled else 2
    finally:
        driver = getattr(gimbal, "driver", None)
        is_connected = getattr(driver, "is_connected", None)
        if callable(is_connected) and is_connected():
            gimbal.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate a calibrated theta by clicking a lower-camera target and watching the gimbal camera while settling."
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
    parser.add_argument("--settle-threshold", type=float, default=DEFAULT_SETTLE_THRESHOLD)
    parser.add_argument("--settle-hold-seconds", type=float, default=DEFAULT_SETTLE_HOLD_SECONDS)
    parser.add_argument("--settle-timeout-seconds", type=float, default=DEFAULT_SETTLE_TIMEOUT_SECONDS)
    parser.add_argument("--overlay-scale", type=float, default=DEFAULT_OVERLAY_SCALE)
    parser.add_argument("--keep-view-after-settle", dest="keep_view_after_settle", action="store_true")
    parser.add_argument("--no-keep-view-after-settle", dest="keep_view_after_settle", action="store_false")
    parser.set_defaults(keep_view_after_settle=DEFAULT_KEEP_VIEW_AFTER_SETTLE)
    parser.add_argument("--release-gimbal-before-view", dest="release_gimbal_before_view", action="store_true")
    parser.add_argument("--no-release-gimbal-before-view", dest="release_gimbal_before_view", action="store_false")
    parser.set_defaults(release_gimbal_before_view=DEFAULT_RELEASE_GIMBAL_BEFORE_VIEW)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(f"[Validate][Error] {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
