import struct
import sys
from pathlib import Path

import pytest


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

import main_tracking_v9 as tracking  # noqa: E402


class _BoundSocket:
    def __init__(self):
        self.bind_calls = []
        self.timeout = None
        self.closed = False

    def bind(self, address):
        self.bind_calls.append(address)

    def settimeout(self, timeout):
        self.timeout = timeout

    def close(self):
        self.closed = True


def test_strike_sender_binds_configured_source_port(monkeypatch):
    sock = _BoundSocket()
    monkeypatch.setattr(tracking.socket, "socket", lambda *_args: sock)

    sender = tracking.StrikeSender(
        "192.168.0.80",
        10123,
        source_port=9630,
    )

    assert sender.source_port == 9630
    assert sock.bind_calls == [("0.0.0.0", 9630)]
    assert sock.timeout == 0.2


def test_strike_sender_rejects_invalid_source_port():
    with pytest.raises(ValueError, match="source port"):
        tracking.StrikeSender("192.168.0.80", 10123, source_port=65536)


def test_strike_packet_appends_encoded_longitude_and_latitude():
    packet = tracking.StrikeSender.build_packet(
        target_id=37,
        distance_m=123.4,
        azimuth_deg=359.9,
        elevation_deg=-12.3,
        longitude_deg=103.456123,
        latitude_deg=27.950686,
    )

    assert len(packet) == 21
    fields = struct.unpack("!2sBBHHhii", packet[:18])
    assert fields == (
        b"\xAA\x55",
        21,
        37,
        1234,
        3599,
        -123,
        -570_767_296,
        1_006_200_000,
    )
    assert packet[18] == tracking.StrikeSender._xor_checksum(packet[:18])
    assert packet[19:] == b"\x55\xAA"


def test_coordinate_encoding_uses_360000_scale_and_signed_int32_wrap():
    assert tracking.StrikeSender._encode_coordinate(103.456123) == -570_767_296
    assert tracking.StrikeSender._encode_coordinate(-12.349) == -444_240_000


def test_strike_packet_accepts_full_rid_elevation_range():
    packet = tracking.StrikeSender.build_packet(
        target_id=1,
        distance_m=89.0,
        azimuth_deg=321.7,
        elevation_deg=87.6,
        longitude_deg=107.1043,
        latitude_deg=27.9507,
    )

    assert struct.unpack("!2sBBHHhii", packet[:18])[5] == 876


def _station_position_snapshot():
    return {
        "valid": True,
        "longitude": 103.456123,
        "latitude": 27.950686,
        "source": "um980",
    }


def test_strike_coordinates_require_a_valid_station_gps_position():
    assert tracking.strike_station_coordinates(_station_position_snapshot()) == (
        103.456123,
        27.950686,
    )

    invalid_position = _station_position_snapshot()
    invalid_position["valid"] = False
    assert tracking.strike_station_coordinates(invalid_position) is None

    configured_default = _station_position_snapshot()
    configured_default["source"] = "configured_default"
    assert tracking.strike_station_coordinates(configured_default) is None

    invalid_longitude = _station_position_snapshot()
    invalid_longitude["longitude"] = 181.0
    assert tracking.strike_station_coordinates(invalid_longitude) is None

    invalid_latitude = _station_position_snapshot()
    invalid_latitude["latitude"] = float("nan")
    assert tracking.strike_station_coordinates(invalid_latitude) is None


class _CaptureStrikeSender:
    def __init__(self):
        self.calls = []

    def send_target_with_timestamp(self, **values):
        self.calls.append(values)
        return b"packet", 100.0


def test_periodic_sender_forwards_coordinates_from_snapshot():
    sender = _CaptureStrikeSender()
    hardware = tracking.SharedHardwareState()
    hardware.is_settled = True
    hardware.is_stationary = True
    hardware.settled_track_id = 91
    worker = tracking.PeriodicStrikeSender(
        sender,
        send_hz=10.0,
        hardware_state=hardware,
    )
    worker.publish({
        "internal_track_id": 91,
        "target_id": 2,
        "distance_m": 123.4,
        "azimuth_deg": 359.9,
        "elevation_deg": -12.3,
        "longitude_deg": 103.456123,
        "latitude_deg": 27.950686,
        "valid_until": 101.0,
    })

    assert worker.send_once(now=100.0) is True
    assert sender.calls == [{
        "target_id": 2,
        "distance_m": 123.4,
        "azimuth_deg": 359.9,
        "elevation_deg": -12.3,
        "longitude_deg": 103.456123,
        "latitude_deg": 27.950686,
    }]


def test_periodic_sender_rejects_target_ids_outside_one_and_two():
    sender = _CaptureStrikeSender()
    hardware = tracking.SharedHardwareState()
    hardware.is_settled = True
    hardware.is_stationary = True
    hardware.settled_track_id = 91
    worker = tracking.PeriodicStrikeSender(
        sender,
        send_hz=10.0,
        hardware_state=hardware,
    )
    worker.publish({
        "internal_track_id": 91,
        "target_id": 3,
        "distance_m": 123.4,
        "azimuth_deg": 359.9,
        "elevation_deg": -12.3,
        "longitude_deg": 103.456123,
        "latitude_deg": 27.950686,
        "valid_until": 101.0,
    })

    assert worker.send_once(now=100.0) is False
    assert sender.calls == []


def test_periodic_sender_allows_visual_ids_when_explicitly_configured():
    sender = _CaptureStrikeSender()
    hardware = tracking.SharedHardwareState()
    hardware.is_settled = True
    hardware.is_stationary = True
    hardware.settled_track_id = 91
    worker = tracking.PeriodicStrikeSender(
        sender,
        send_hz=10.0,
        hardware_state=hardware,
        allowed_target_ids=(3, 4, 5),
    )
    worker.publish({
        "internal_track_id": 91,
        "target_id": 3,
        "distance_m": 123.4,
        "azimuth_deg": 359.9,
        "elevation_deg": -12.3,
        "longitude_deg": 103.456123,
        "latitude_deg": 27.950686,
        "valid_until": 101.0,
    })

    assert worker.send_once(now=100.0) is True
    assert [call["target_id"] for call in sender.calls] == [3]


def _direct_snapshot(target_id, valid_until=101.0):
    return {
        "target_id": target_id,
        "distance_m": 100.0 + target_id,
        "azimuth_deg": 170.0 + target_id,
        "elevation_deg": 8.0 + target_id,
        "longitude_deg": 107.1043,
        "latitude_deg": 27.9507,
        "valid_until": valid_until,
    }


def test_periodic_sender_repeats_each_special_rid_target_without_gimbal_gate():
    sender = _CaptureStrikeSender()
    worker = tracking.PeriodicStrikeSender(
        sender,
        send_hz=10.0,
        hardware_state=None,
    )
    worker.publish(_direct_snapshot(1))
    worker.publish(_direct_snapshot(2))

    assert worker.send_once(now=100.0) is True
    assert worker.send_once(now=100.1) is True
    assert [call["target_id"] for call in sender.calls] == [1, 2, 1, 2]


def test_periodic_sender_replace_removes_targets_missing_from_latest_ui_batch():
    sender = _CaptureStrikeSender()
    worker = tracking.PeriodicStrikeSender(
        sender,
        send_hz=10.0,
        hardware_state=None,
    )
    worker.replace([_direct_snapshot(1), _direct_snapshot(2)])
    assert worker.send_once(now=100.0) is True

    worker.replace([_direct_snapshot(2)])
    assert worker.send_once(now=100.1) is True

    assert [call["target_id"] for call in sender.calls] == [1, 2, 2]
