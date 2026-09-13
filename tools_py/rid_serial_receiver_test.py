#!/usr/bin/env python3
"""Standalone XP-MRID-04 strict serial receiver and parser test.

The protocol implementation is not duplicated here. This tool imports
``RIDStreamParser`` directly from ``core/rid_tracking.py``, so live testing and
the production tracking service always use the same header/length/tail parser.

Examples:
    python tools_py/rid_serial_receiver_test.py --list-ports
    python tools_py/rid_serial_receiver_test.py --port COM13
    python tools_py/rid_serial_receiver_test.py --port COM13 --show-raw
    python tools_py/rid_serial_receiver_test.py --port COM13 --once
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CORE_DIR = PROJECT_ROOT / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from rid_tracking import RIDStreamParser, is_valid_rid_position  # noqa: E402


PARSER_COUNTER_NAMES = (
    "discarded_bytes",
    "decode_errors",
    "valid_frames",
    "header_errors",
    "length_errors",
    "tail_errors",
    "truncated_frames",
    "utf8_errors",
    "json_errors",
    "json_type_errors",
)

ERROR_COUNTER_NAMES = (
    "header_errors",
    "length_errors",
    "tail_errors",
    "truncated_frames",
    "utf8_errors",
    "json_errors",
    "json_type_errors",
)


def _counter_snapshot(parser):
    return {
        name: int(getattr(parser, name))
        for name in PARSER_COUNTER_NAMES
    }


def _counter_delta(before, after):
    return {name: after[name] - before[name] for name in PARSER_COUNTER_NAMES}


def _rid_summary(payload):
    uav = payload.get("UAVInfo") if isinstance(payload, dict) else None
    uav = uav if isinstance(uav, dict) else {}
    monitor = payload.get("MonitorInfo") if isinstance(payload, dict) else None
    monitor = monitor if isinstance(monitor, dict) else {}
    position_valid = is_valid_rid_position(uav.get("Lon"), uav.get("Lat"))
    return (
        f"module={monitor.get('Name', '')!s} "
        f"rid_id={uav.get('ID', '')!s} "
        f"standard={uav.get('RID_Standard', '')!s} "
        f"position_valid={position_valid} "
        f"lat={uav.get('Lat', '')!s} lon={uav.get('Lon', '')!s} "
        f"alt_geo={uav.get('AltGeo', '')!s} "
        f"height={uav.get('Height', '')!s} "
        f"h_speed={uav.get('H_Speed', '')!s} "
        f"v_speed={uav.get('V_Speed', '')!s} "
        f"rid_ts={uav.get('T_Stamp', '')!s}"
    )


class TestLogs:
    def __init__(self, log_dir):
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.raw_path = log_dir / f"raw_rid_serial_test_{timestamp}.jsonl"
        self.decoded_path = log_dir / f"decoded_rid_test_{timestamp}.jsonl"
        self.raw_file = self.raw_path.open("a", encoding="utf-8", newline="\n")
        self.decoded_file = self.decoded_path.open(
            "a", encoding="utf-8", newline="\n"
        )

    @staticmethod
    def _write(file_obj, row):
        file_obj.write(
            json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
        )
        file_obj.flush()

    def write_raw(self, row):
        self._write(self.raw_file, row)

    def write_decoded(self, row):
        self._write(self.decoded_file, row)

    def close(self):
        for file_obj in (self.raw_file, self.decoded_file):
            try:
                file_obj.flush()
                file_obj.close()
            except Exception:
                pass


def _build_raw_log_row(receive_ts, chunk, parser, delta, frames, payload_count):
    return {
        "receive_ts": receive_ts,
        "byte_count": len(chunk),
        "chunk_hex": chunk.hex(),
        "parsed_payload_count": payload_count,
        "decoded_length_fields": [item["length_field"] for item in frames],
        "decoded_payload_lengths": [item["payload_length"] for item in frames],
        "decoded_frame_lengths": [item["frame_length"] for item in frames],
        "discarded_bytes_delta": delta["discarded_bytes"],
        "discarded_bytes_total": parser.discarded_bytes,
        "decode_errors_delta": delta["decode_errors"],
        "decode_errors_total": parser.decode_errors,
        "valid_frames_delta": delta["valid_frames"],
        "valid_frames_total": parser.valid_frames,
        "header_errors_delta": delta["header_errors"],
        "header_errors_total": parser.header_errors,
        "length_errors_delta": delta["length_errors"],
        "length_errors_total": parser.length_errors,
        "tail_errors_delta": delta["tail_errors"],
        "tail_errors_total": parser.tail_errors,
        "truncated_frames_delta": delta["truncated_frames"],
        "truncated_frames_total": parser.truncated_frames,
        "utf8_errors_delta": delta["utf8_errors"],
        "utf8_errors_total": parser.utf8_errors,
        "json_errors_delta": delta["json_errors"],
        "json_errors_total": parser.json_errors,
        "json_type_errors_delta": delta["json_type_errors"],
        "json_type_errors_total": parser.json_type_errors,
        "parser_buffer_bytes_after": len(parser.buffer),
    }


def build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=os.getenv("RID_PORT", "").strip(),
        help="RID serial port, for example COM13; defaults to RID_PORT",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=int(os.getenv("RID_BAUDRATE", "115200")),
        help="serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.20,
        help="serial read timeout in seconds (default: 0.20)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="stop after the first successfully decoded RID frame",
    )
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="print every serial read chunk as hexadecimal",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print one RID summary line without pretty-printing the full JSON",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="do not create raw/decoded JSONL test logs",
    )
    parser.add_argument(
        "--log-dir",
        default=str(PROJECT_ROOT / "logs" / "rid_serial_test"),
        help="test log directory",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="list detected serial ports and exit",
    )
    return parser


def _print_ports(list_ports):
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return
    print("Detected serial ports:")
    for item in ports:
        print(
            f"  {item.device}: {item.description} "
            f"[hwid={item.hwid}]"
        )


def run(args):
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as exc:
        print(
            "pyserial is required. Install it with: python -m pip install pyserial",
            file=sys.stderr,
        )
        return 2

    if args.list_ports:
        _print_ports(list_ports)
        return 0
    if not args.port:
        print(
            "RID serial port is required. Use --port COMx or set RID_PORT.\n"
            "Run with --list-ports to inspect available ports.",
            file=sys.stderr,
        )
        return 2
    if args.baud <= 0 or args.timeout < 0.0 or args.duration < 0.0:
        print("Invalid baud/timeout/duration argument.", file=sys.stderr)
        return 2

    stream_parser = RIDStreamParser()
    logs = None if args.no_log else TestLogs(args.log_dir)
    if logs is not None:
        print(f"Raw log:     {logs.raw_path.resolve()}")
        print(f"Decoded log: {logs.decoded_path.resolve()}")

    print(
        f"Opening RID serial {args.port}@{args.baud} 8N1, "
        "strict frame=08 17 30 + LEN_LE + JSON + 3F 55"
    )
    print("Press Ctrl+C to stop.")
    start_monotonic = time.monotonic()
    received_bytes = 0

    try:
        with serial.Serial(
            args.port,
            args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        ) as ser:
            while True:
                if (
                    args.duration > 0.0
                    and time.monotonic() - start_monotonic >= args.duration
                ):
                    break
                waiting = int(getattr(ser, "in_waiting", 0) or 0)
                chunk = ser.read(max(1, min(waiting, 65536)))
                if not chunk:
                    continue

                receive_ts = time.time()
                received_bytes += len(chunk)
                before = _counter_snapshot(stream_parser)
                payloads = stream_parser.feed(chunk)
                after = _counter_snapshot(stream_parser)
                delta = _counter_delta(before, after)
                frames = list(stream_parser.last_feed_frames)

                raw_row = _build_raw_log_row(
                    receive_ts,
                    chunk,
                    stream_parser,
                    delta,
                    frames,
                    len(payloads),
                )
                if logs is not None:
                    logs.write_raw(raw_row)
                if args.show_raw:
                    print(
                        f"[{receive_ts:.6f}] RAW bytes={len(chunk)} "
                        f"hex={chunk.hex()}"
                    )

                errors = {
                    name: delta[name]
                    for name in ERROR_COUNTER_NAMES
                    if delta[name]
                }
                if errors:
                    print(
                        f"[{receive_ts:.6f}] PARSE_ERROR errors={errors} "
                        f"resync_discarded={delta['discarded_bytes']} "
                        f"buffer_after={len(stream_parser.buffer)}"
                    )

                for index, payload in enumerate(payloads):
                    frame = frames[index] if index < len(frames) else {}
                    print(
                        f"[{receive_ts:.6f}] RID_FRAME_OK "
                        f"length_field={frame.get('length_field', '')} "
                        f"json_bytes={frame.get('payload_length', '')} "
                        f"frame_bytes={frame.get('frame_length', '')}"
                    )
                    print(f"  {_rid_summary(payload)}")
                    if not args.summary_only:
                        print(json.dumps(payload, ensure_ascii=False, indent=2))
                    if logs is not None:
                        logs.write_decoded({
                            "receive_ts": receive_ts,
                            **frame,
                            "payload": payload,
                        })

                if args.once and payloads:
                    break
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 2
    finally:
        if logs is not None:
            logs.close()

    elapsed = max(0.0, time.monotonic() - start_monotonic)
    final = _counter_snapshot(stream_parser)
    print(
        "RID serial test summary: "
        f"elapsed={elapsed:.3f}s bytes={received_bytes} "
        f"valid_frames={final['valid_frames']} "
        f"header_errors={final['header_errors']} "
        f"length_errors={final['length_errors']} "
        f"tail_errors={final['tail_errors']} "
        f"truncated_frames={final['truncated_frames']} "
        f"utf8_errors={final['utf8_errors']} "
        f"json_errors={final['json_errors']} "
        f"json_type_errors={final['json_type_errors']} "
        f"resync_discarded_bytes={final['discarded_bytes']} "
        f"buffer_after={len(stream_parser.buffer)}"
    )
    return 0


def main(argv=None):
    args = build_argument_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
