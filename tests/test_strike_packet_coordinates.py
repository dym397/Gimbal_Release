import struct
import sys
from pathlib import Path


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

import main_tracking_v9 as tracking  # noqa: E402


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
        37_242_000,
        10_062_000,
    )
    assert packet[18] == tracking.StrikeSender._xor_checksum(packet[:18])
    assert packet[19:] == b"\x55\xAA"


def test_coordinate_encoding_truncates_toward_zero_before_arcsecond_scale():
    assert tracking.StrikeSender._encode_coordinate(103.456123) == 37_242_000
    assert tracking.StrikeSender._encode_coordinate(-12.349) == -4_442_400


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
