#!/usr/bin/env python3
"""
Deterministic UDP target generator for main_tracking_v9.py SORT/Kalman tests.

This script intentionally tests tracking logic, not detector quality. It sends
payloads already compatible with parse_udp_objects():
{
    "board": "BOARD_1",
    "cam": 2,
    "objs": [[x1, y1, x2, y2, distance]]
}
"""

import argparse
import json
import math
import random
import socket
import time
from typing import Iterable, List, Sequence, Tuple


IMG_W = 2560.0
IMG_H = 1440.0


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def round2(v: float) -> float:
    return round(float(v), 2)


def make_box(cx: float, cy: float, w: float, h: float) -> List[float]:
    half_w = w / 2.0
    half_h = h / 2.0
    cx = clamp(cx, half_w, IMG_W - 1.0 - half_w)
    cy = clamp(cy, half_h, IMG_H - 1.0 - half_h)
    return [
        round2(cx - half_w),
        round2(cy - half_h),
        round2(cx + half_w),
        round2(cy + half_h),
        # distance is appended by build_packet()
    ]


def box_with_distance(cx: float, cy: float, args: argparse.Namespace, distance: float) -> List[float]:
    box = make_box(cx, cy, args.box_w, args.box_h)
    return [box[0], box[1], box[2], box[3], round2(distance)]


def should_drop_frame(seq: int, args: argparse.Namespace) -> bool:
    if args.mode != "drop_frames" or args.drop_every <= 0 or args.drop_count <= 0:
        return False
    if seq < args.drop_every:
        return False
    pos = seq % args.drop_every
    return 0 <= pos < args.drop_count


def single_center(seq: int, elapsed: float, args: argparse.Namespace) -> Tuple[float, float]:
    cx = args.cx
    cy = args.cy

    if args.mode == "static_single":
        pass
    elif args.mode == "jitter_single":
        cx += random.uniform(-args.jitter_px, args.jitter_px)
        cy += random.uniform(-args.jitter_px, args.jitter_px)
    elif args.mode == "constant_speed":
        cx += args.speed_px * elapsed
        cy += args.vy_px * elapsed
    elif args.mode == "sudden_turn":
        turn_t = args.turn_after
        if elapsed <= turn_t:
            cx += args.speed_px * elapsed
            cy += args.vy_px * elapsed
        else:
            cx += args.speed_px * turn_t - args.speed_px * (elapsed - turn_t)
            cy += args.vy_px * turn_t - args.vy_px * (elapsed - turn_t)
    elif args.mode == "drop_frames":
        cx += args.speed_px * elapsed
        cy += args.vy_px * elapsed
    else:
        raise ValueError(f"unsupported single-target mode: {args.mode}")

    return cx, cy


def scenario_targets(seq: int, elapsed: float, args: argparse.Namespace) -> Sequence[Tuple[float, float, float]]:
    if args.mode in ("static_single", "jitter_single", "constant_speed", "sudden_turn", "drop_frames"):
        cx, cy = single_center(seq, elapsed, args)
        return [(cx, cy, args.distance_m)]

    if args.mode == "two_crossing":
        center_y = args.cy
        left_start = args.cross_margin
        right_start = IMG_W - args.cross_margin
        speed = abs(args.speed_px)
        cx_a = left_start + speed * elapsed
        cx_b = right_start - speed * elapsed
        return [
            (cx_a, center_y - args.cross_y_offset, args.distance_m),
            (cx_b, center_y + args.cross_y_offset, args.distance_m + 40.0),
        ]

    if args.mode == "multi_stable":
        # Three stable, separated targets with mild deterministic motion.
        return [
            (800.0 + 80.0 * math.sin(elapsed * 0.7), 600.0, args.distance_m),
            (1280.0 + 60.0 * math.sin(elapsed * 0.5 + 1.4), 720.0, args.distance_m + 60.0),
            (1760.0 + 70.0 * math.sin(elapsed * 0.9 + 2.0), 840.0, args.distance_m + 100.0),
        ]

    raise ValueError(f"unsupported mode: {args.mode}")


def build_packet(seq: int, elapsed: float, args: argparse.Namespace) -> dict:
    objs = [
        box_with_distance(cx, cy, args, distance)
        for cx, cy, distance in scenario_targets(seq, elapsed, args)
    ]
    return {
        "board": args.board,
        "cam": args.cam,
        "objs": objs,
        "seq": seq,
        "mode": args.mode,
        "ts_sender": time.time(),
        "elapsed_sender": round2(elapsed),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock UDP sender for SORT/Kalman tracking tests")
    parser.add_argument(
        "--mode",
        choices=(
            "static_single",
            "jitter_single",
            "constant_speed",
            "sudden_turn",
            "drop_frames",
            "two_crossing",
            "multi_stable",
        ),
        default="static_single",
    )
    parser.add_argument("--replay-jsonl", default="", help="Replay a raw_udp_*.jsonl capture instead of generated packets")
    parser.add_argument("--replay-speed", type=float, default=1.0, help="Replay speed multiplier for --replay-jsonl")
    parser.add_argument("--replay-max-gap", type=float, default=1.0, help="Maximum sleep gap in seconds while replaying")
    parser.add_argument("--replay-start-ts", type=float, default=None, help="Only replay packets with recv_ts >= this value")
    parser.add_argument("--replay-end-ts", type=float, default=None, help="Only replay packets with recv_ts <= this value")
    parser.add_argument("--limit", type=int, default=0, help="Maximum packets to send while replaying; 0 means no limit")
    parser.add_argument("--ip", default="127.0.0.1", help="Receiver IP")
    parser.add_argument("--port", type=int, default=8888, help="Receiver UDP port")
    parser.add_argument("--board", default="BOARD_1", help="Board id")
    parser.add_argument("--cam", type=int, default=2, help="Camera index")
    parser.add_argument("--fps", type=float, default=15.0, help="Send rate in Hz")
    parser.add_argument("--duration", type=float, default=30.0, help="Duration in seconds")
    parser.add_argument("--cx", type=float, default=1280.0, help="Initial target center x")
    parser.add_argument("--cy", type=float, default=720.0, help="Initial target center y")
    parser.add_argument("--box-w", type=float, default=120.0, help="Bounding box width")
    parser.add_argument("--box-h", type=float, default=80.0, help="Bounding box height")
    parser.add_argument("--distance-m", type=float, default=300.0, help="Mono distance in meters")
    parser.add_argument("--jitter-px", type=float, default=20.0, help="Jitter amplitude for jitter_single")
    parser.add_argument("--speed-px", type=float, default=80.0, help="Horizontal speed in px/s")
    parser.add_argument("--vy-px", type=float, default=0.0, help="Vertical speed in px/s")
    parser.add_argument("--turn-after", type=float, default=8.0, help="Turn time for sudden_turn")
    parser.add_argument("--drop-every", type=int, default=60, help="Drop cycle length in frames")
    parser.add_argument("--drop-count", type=int, default=10, help="Dropped frames per cycle")
    parser.add_argument("--cross-margin", type=float, default=600.0, help="two_crossing start margin")
    parser.add_argument("--cross-y-offset", type=float, default=60.0, help="two_crossing vertical separation")
    parser.add_argument("--dry-run", action="store_true", help="Print packets instead of sending")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.replay_jsonl:
        if args.replay_speed <= 0:
            raise ValueError("replay-speed must be > 0")
        if args.replay_max_gap < 0:
            raise ValueError("replay-max-gap must be >= 0")
        if args.limit < 0:
            raise ValueError("limit must be >= 0")
        return
    if args.fps <= 0:
        raise ValueError("fps must be > 0")
    if args.duration <= 0:
        raise ValueError("duration must be > 0")
    if args.box_w <= 0 or args.box_h <= 0:
        raise ValueError("box size must be > 0")
    if args.distance_m <= 0:
        raise ValueError("distance-m must be > 0")
    if args.jitter_px < 0:
        raise ValueError("jitter-px must be >= 0")
    if args.drop_every < 0 or args.drop_count < 0:
        raise ValueError("drop values must be >= 0")
    if args.drop_every > 0 and args.drop_count >= args.drop_every:
        raise ValueError("drop-count must be smaller than drop-every")


def replay_jsonl(args: argparse.Namespace) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    skipped = 0
    first_recv_ts = None
    replay_start_t = time.time()

    print(
        f"[MockSender][Replay] target={args.ip}:{args.port}, "
        f"jsonl={args.replay_jsonl}, speed={args.replay_speed}, max_gap={args.replay_max_gap}"
    )

    try:
        with open(args.replay_jsonl, "r", encoding="utf-8-sig") as f:
            for line_no, line in enumerate(f, start=1):
                if args.limit and sent >= args.limit:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError as exc:
                    skipped += 1
                    print(f"[MockSender][Replay][Warn] skip line={line_no}, json error={exc}")
                    continue

                raw_objs = rec.get("raw_objs", [])
                if not isinstance(raw_objs, list):
                    skipped += 1
                    continue
                recv_ts = rec.get("recv_ts")
                try:
                    recv_ts = float(recv_ts)
                except (TypeError, ValueError):
                    recv_ts = None
                if recv_ts is not None and args.replay_start_ts is not None and recv_ts < args.replay_start_ts:
                    continue
                if recv_ts is not None and args.replay_end_ts is not None and recv_ts > args.replay_end_ts:
                    break

                if recv_ts is not None:
                    if first_recv_ts is None:
                        first_recv_ts = recv_ts
                    target_elapsed = (recv_ts - first_recv_ts) / args.replay_speed
                    sleep_time = replay_start_t + target_elapsed - time.time()
                    if args.replay_max_gap > 0:
                        sleep_time = min(sleep_time, args.replay_max_gap)
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                packet = {
                    "board": rec.get("board", args.board),
                    "cam": rec.get("cam", args.cam),
                    "objs": raw_objs,
                    "seq": rec.get("seq", ""),
                    "mode": rec.get("mode", "replay_jsonl") or "replay_jsonl",
                    "ts_sender": time.time(),
                    "replay_recv_ts": recv_ts,
                }
                raw = json.dumps(packet, ensure_ascii=False).encode("utf-8")
                if args.dry_run:
                    print(raw.decode("utf-8"))
                else:
                    sock.sendto(raw, (args.ip, args.port))
                sent += 1
                if sent == 1 or sent % 50 == 0:
                    print(
                        f"[MockSender][Replay] sent={sent}, line={line_no}, "
                        f"objs={len(raw_objs)}, board={packet['board']}, cam={packet['cam']}"
                    )
    finally:
        sock.close()

    print(f"[MockSender][Replay] done, sent={sent}, skipped={skipped}")


def main() -> None:
    args = parse_args()
    validate_args(args)
    if args.replay_jsonl:
        replay_jsonl(args)
        return

    interval = 1.0 / args.fps
    total_frames = max(1, int(round(args.duration * args.fps)))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    dropped = 0
    t0 = time.time()

    print(
        f"[MockSender] target={args.ip}:{args.port}, mode={args.mode}, "
        f"board={args.board}, cam={args.cam}, fps={args.fps}, frames={total_frames}"
    )

    try:
        for seq in range(total_frames):
            elapsed = seq * interval
            if should_drop_frame(seq, args):
                dropped += 1
                if dropped == 1 or dropped % max(1, int(args.fps)) == 0:
                    print(f"[MockSender] drop seq={seq}, dropped={dropped}")
            else:
                packet = build_packet(seq, elapsed, args)
                raw = json.dumps(packet, ensure_ascii=False).encode("utf-8")
                if args.dry_run:
                    print(raw.decode("utf-8"))
                else:
                    sock.sendto(raw, (args.ip, args.port))
                sent += 1
                if sent == 1 or sent % max(1, int(args.fps)) == 0:
                    centers = [
                        (round2((obj[0] + obj[2]) / 2.0), round2((obj[1] + obj[3]) / 2.0))
                        for obj in packet["objs"]
                    ]
                    print(f"[MockSender] sent={sent}, seq={seq}, centers={centers}")

            next_deadline = t0 + (seq + 1) * interval
            sleep_time = next_deadline - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        sock.close()

    print(f"[MockSender] done, sent={sent}, dropped={dropped}")


if __name__ == "__main__":
    main()
