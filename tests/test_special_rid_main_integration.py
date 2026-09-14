import csv
import struct
import sys
from pathlib import Path


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

import main_tracking_v9 as tracking  # noqa: E402
from special_rid_identity import (  # noqa: E402
    SortGeneration,
    should_send_sort_through_ordinary_ui,
    sort_observation_from_track,
)


class _Track:
    def __init__(self, sort_id=238, created_ts=100.5):
        self.id = sort_id
        self.created_ts = created_ts
        self.ui_confirmed = True
        self.state = _State()
        self.last_source_board = "BOARD_3"
        self.last_source_cam = 2
        self.last_source_logic_id = 13
        self.last_update_ts = 101.0
        self.hit_streak = 20


class _State:
    def __getitem__(self, index):
        values = {
            (0, 0): 12.0,
            (1, 0): 8.0,
            (2, 0): 1.5,
            (3, 0): -0.5,
        }
        return values[index]


def test_owned_special_sort_is_suppressed_from_ordinary_ui_path():
    track = _Track()
    owned = {SortGeneration(238, 100.5)}

    assert should_send_sort_through_ordinary_ui(track, owned) is False


def test_unowned_sort_keeps_existing_ordinary_ui_path():
    assert should_send_sort_through_ordinary_ui(_Track(), set()) is True


def test_ordinary_ui_allocator_skips_reserved_special_ids():
    assert tracking.next_unreserved_ui_id(1, frozenset((1, 2))) == 3
    assert tracking.next_unreserved_ui_id(3, frozenset((1, 2))) == 3


def test_special_rid_exclusive_mode_blocks_all_ordinary_ui_tracks():
    track = _Track()

    selected = tracking.select_ordinary_ui_tracks_for_output(
        [track],
        owned_special_generations=set(),
        special_rid_ui_exclusive=True,
    )

    assert selected == []


def test_ordinary_ui_tracks_remain_available_when_exclusive_mode_is_off():
    track = _Track()

    selected = tracking.select_ordinary_ui_tracks_for_output(
        [track],
        owned_special_generations=set(),
        special_rid_ui_exclusive=False,
    )

    assert selected == [track]


def test_ui_and_strike_tracks_become_confirmed_on_seventh_hit():
    track = _Track()
    track.hit_streak = 6
    track.ui_confirmed = False
    track.strike_confirmed = False

    tracking.update_track_confirmation_flags(track)
    assert track.ui_confirmed is False
    assert track.strike_confirmed is False

    track.hit_streak = 7
    tracking.update_track_confirmation_flags(track)
    assert track.ui_confirmed is True
    assert track.strike_confirmed is True


def test_track_conversion_preserves_exact_generation_and_camera_source():
    observation = sort_observation_from_track(
        _Track(), map_azimuth=lambda relative: (relative + 180.0) % 360.0
    )

    assert observation.generation == SortGeneration(238, 100.5)
    assert observation.map_az == 192.0
    assert (observation.board, observation.camera_id, observation.logic_id) == (
        "BOARD_3",
        2,
        13,
    )


class _Socket:
    def __init__(self):
        self.packet = None

    def sendto(self, packet, destination):
        self.packet = packet
        return len(packet)


def test_special_status_uses_existing_34_byte_ui_packet_contract():
    sender = tracking.UISender("127.0.0.1", 9999)
    fake_socket = _Socket()
    sender.sock.close()
    sender.sock = fake_socket

    assert sender.send_status(
        board_str="BOARD_3",
        camera_id=2,
        target_id=1,
        azimuth=170.0,
        elevation=9.0,
        distance=155.0,
        threat_score=50.0,
        replaced_target_id=0,
    )

    assert len(fake_socket.packet) == 34
    assert struct.unpack("!BB8sIffffI", fake_socket.packet)[8] == 0


def test_field_logger_writes_dedicated_special_identity_csv(tmp_path):
    logger = tracking.FieldLogger(tmp_path)
    logger.write_special_rid_identity({
        "timestamp": "10.000000",
        "event": "REGISTERED",
        "rid_id": "RID-A",
        "ui_id": 1,
    })
    logger.close()

    paths = list(tmp_path.glob("special_rid_identity_*.csv"))
    assert len(paths) == 1
    with paths[0].open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["event"] == "REGISTERED"
    assert rows[0]["ui_id"] == "1"
