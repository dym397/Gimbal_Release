import argparse
import json
import socket
import time


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 1234
BUFFER_SIZE = 4096


def format_value(value):
    if value is None:
        return "null"
    return str(value)


def parse_json_packet(data):
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        return f"invalid UTF-8 data: {exc}; raw={data.hex(' ')}"

    try:
        packet = json.loads(text)
    except json.JSONDecodeError as exc:
        return f"invalid JSON: {exc}; text={text!r}"

    if not isinstance(packet, dict):
        return f"JSON is not an object: {packet!r}"

    height = packet.get("height")
    distance = packet.get("distance")
    status = packet.get("status")
    timestamp = packet.get("timestamp")

    return (
        "JSON "
        f"height={format_value(height)} m, "
        f"distance={format_value(distance)} m, "
        f"status={format_value(status)}, "
        f"timestamp={format_value(timestamp)}"
    )


def main():
    parser = argparse.ArgumentParser(description="UDP receiver for UTF-8 JSON telemetry packets")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"bind host, default: {DEFAULT_HOST}")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"bind UDP port, default: {DEFAULT_PORT}")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))

    print(f"[UDPJsonReceiver] Listening on {args.host}:{args.port}")
    print("[UDPJsonReceiver] Press Ctrl+C to stop")

    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        recv_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(f"[{recv_time}] from {addr[0]}:{addr[1]} {parse_json_packet(data)}", flush=True)


if __name__ == "__main__":
    main()
