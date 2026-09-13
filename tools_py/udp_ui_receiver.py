import argparse
import socket
import struct
import time


MSG_STATUS = 0x02
MSG_GPS = 0x03


def parse_status_packet(data):
    expected_len = struct.calcsize("!BB8sIffffI")
    if len(data) != expected_len:
        return f"invalid status packet length={len(data)}, expected={expected_len}"

    (
        msg_type,
        camera_id,
        board_bytes,
        target_id,
        azimuth,
        elevation,
        distance,
        threat_score,
        replaced_target_id,
    ) = struct.unpack("!BB8sIffffI", data)
    board = board_bytes.rstrip(b"\x00").decode("utf-8", errors="replace")
    return (
        f"STATUS type=0x{msg_type:02X}, board={board}, camera_id={camera_id}, "
        f"target_id={target_id}, az={azimuth:.3f}, el={elevation:.3f}, "
        f"dist={distance:.3f}, threat={threat_score:.3f}, "
        f"replaced_target_id={replaced_target_id}"
    )


def parse_gps_packet(data):
    expected_len = struct.calcsize("!Bff")
    if len(data) != expected_len:
        return f"invalid GPS packet length={len(data)}, expected={expected_len}"

    msg_type, latitude, longitude = struct.unpack("!Bff", data)
    return f"GPS type=0x{msg_type:02X}, latitude={latitude:.6f}, longitude={longitude:.6f}"


def parse_packet(data):
    if not data:
        return "empty packet"

    msg_type = data[0]
    if msg_type == MSG_STATUS:
        return parse_status_packet(data)
    if msg_type == MSG_GPS:
        return parse_gps_packet(data)
    return f"unknown packet type=0x{msg_type:02X}, length={len(data)}, hex={data.hex(' ')}"


def main():
    parser = argparse.ArgumentParser(description="UDP UI receiver simulator for main_tracking_v9.py")
    parser.add_argument("--host", default="0.0.0.0", help="bind host, default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=9999, help="bind UDP port, default: 9999")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    print(f"[UIReceiver] Listening on {args.host}:{args.port}")

    while True:
        data, addr = sock.recvfrom(4096)
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(f"[{ts}] from {addr[0]}:{addr[1]} {parse_packet(data)}", flush=True)


if __name__ == "__main__":
    main()
