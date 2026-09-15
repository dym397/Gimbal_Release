import csv
import hashlib
import json
import math
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR))

from tools_py.replay_special_rid_identity import replay_run  # noqa: E402


RID_LASER_TEST_1 = "TEST-RID-LASER-1"
RID_LASER_TEST_2 = "TEST-RID-LASER-2"
EARTH_RADIUS_M = 6_371_008.8


def _rid_laser_test_digests():
    return frozenset(
        hashlib.sha256(value.encode("utf-8")).digest()
        for value in (RID_LASER_TEST_1, RID_LASER_TEST_2)
    )


def _rid_record(rid_id, timestamp, azimuth):
    distance = 100.0
    azimuth_rad = math.radians(azimuth)
    north_m = distance * math.cos(azimuth_rad)
    east_m = distance * math.sin(azimuth_rad)
    latitude = 30.0 + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = 104.0 + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(30.0)))
    )
    return {
        "receive_ts": timestamp,
        "payload": {
            "UAVInfo": {
                "RID_Standard": "GB42590-2023",
                "ID": rid_id,
                "ID_Type": 1,
                "Lon": longitude,
                "Lat": latitude,
                "AltGeo": 460.0,
                "Height": 10.0,
                "H_Speed": 0.0,
                "V_Speed": 0.0,
                "Trk": 0.0,
                "T_Stamp": timestamp,
            }
        },
    }


def _write_compact_fixture(path):
    path.mkdir()
    events = [{
        "timestamp": "9.0",
        "event": "GPS_STATION_FIX",
        "track_id": "",
        "meas_idx": "",
        "meas_az": "",
        "meas_el": "",
        "hit_streak": "",
        "reason": "lat=30.0,lon=104.0,alt_msl_m=450.0",
    }]
    measurements = []

    def add_event(timestamp, event, track_id, azimuth, elevation, hit, logic_id):
        events.append({
            "timestamp": f"{timestamp:.6f}",
            "event": event,
            "track_id": track_id,
            "meas_idx": 0,
            "meas_az": azimuth,
            "meas_el": elevation,
            "hit_streak": hit,
            "reason": "",
        })
        measurements.append({
            "timestamp": f"{timestamp:.6f}",
            "board": "BOARD_3",
            "cam": logic_id - 11,
            "logic_id": logic_id,
            "meas_idx": 0,
        })

    add_event(10.0, "MATCH_ACCEPT", 177, 15.0, 3.0, 20, 13)
    add_event(30.0, "NEW_TRACK", 189, 2.0, 10.0, 1, 13)
    for index in range(1, 17):
        add_event(30.0 + index * 0.1, "MATCH_ACCEPT", 189, 5.0, 10.0, index, 13)
    add_event(64.0, "NEW_TRACK", 196, 5.49, 10.0, 1, 13)
    for index in range(1, 17):
        add_event(64.0 + index * 0.1, "MATCH_ACCEPT", 196, 5.49, 10.0, index, 13)
    add_event(70.0, "NEW_TRACK", 202, 12.553, 10.0, 1, 12)
    for index in range(1, 17):
        azimuth = 12.553 + (6.2 - 12.553) * index / 16.0
        add_event(70.0 + index * 0.1, "MATCH_ACCEPT", 202, azimuth, 10.0, index, 12)
    add_event(75.0, "NEW_TRACK", 205, 14.2, 10.0, 1, 13)
    for index in range(1, 17):
        add_event(75.0 + index * 0.1, "MATCH_ACCEPT", 205, 14.2, 10.0, index, 13)
    add_event(110.0, "MATCH_ACCEPT", 205, 14.2, 10.0, 100, 13)
    add_event(110.2, "NEW_TRACK", 238, 14.6, 10.0, 1, 13)
    for index in range(1, 17):
        add_event(110.2 + index * 0.1, "MATCH_ACCEPT", 238, 14.6, 10.0, index, 13)

    with (path / "events_fixture.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=events[0])
        writer.writeheader()
        writer.writerows(events)
    with (path / "measurements_fixture.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=measurements[0])
        writer.writeheader()
        writer.writerows(measurements)
    rid_records = [
        _rid_record(RID_LASER_TEST_1, 9.9, 195.0),
        _rid_record(RID_LASER_TEST_2, 20.0, 182.0),
        _rid_record(RID_LASER_TEST_2, 29.9, 182.0),
        _rid_record(RID_LASER_TEST_2, 63.9, 185.49),
        _rid_record(RID_LASER_TEST_2, 69.9, 192.553),
        _rid_record(RID_LASER_TEST_2, 75.6, 194.2),
    ]
    with (path / "raw_rid_fixture.jsonl").open("w", encoding="utf-8") as stream:
        for record in rid_records:
            stream.write(json.dumps(record) + "\n")


def test_replay_starts_at_first_rid_laser_checkpoint_and_preserves_second_family(tmp_path):
    fixture_dir = tmp_path / "fixture"
    _write_compact_fixture(fixture_dir)

    result = replay_run(
        fixture_dir,
        start_ts=10.0,
        rid_laser_digests=_rid_laser_test_digests(),
    )

    assert result.registration_order == [RID_LASER_TEST_1, RID_LASER_TEST_2]
    assert result.ui_ids == {RID_LASER_TEST_1: 1, RID_LASER_TEST_2: 2}
    assert {189, 196, 202, 205, 238} <= result.sort_families[RID_LASER_TEST_2]
    assert result.owner_by_sort[238] == RID_LASER_TEST_2
    assert result.forbidden_steal_count >= 1


def test_replay_json_payload_is_serializable(tmp_path):
    fixture_dir = tmp_path / "fixture"
    _write_compact_fixture(fixture_dir)

    result = replay_run(
        fixture_dir,
        start_ts=10.0,
        rid_laser_digests=_rid_laser_test_digests(),
    )
    encoded = json.dumps(result.json_ready(), ensure_ascii=False, sort_keys=True)

    assert f'"{RID_LASER_TEST_1}": [177]' in encoded
