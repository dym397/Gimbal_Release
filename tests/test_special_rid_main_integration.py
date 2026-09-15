import csv
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


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


class _SpecialRidOwnerRegistry:
    def __init__(self, owners):
        self.owners = dict(owners)

    def owner_of(self, generation):
        ui_id = self.owners.get(generation)
        return None if ui_id is None else SimpleNamespace(ui_id=ui_id)


def test_owned_special_sort_is_suppressed_from_ordinary_ui_path():
    track = _Track()
    owned = {SortGeneration(238, 100.5)}

    assert should_send_sort_through_ordinary_ui(track, owned) is False


def test_unowned_sort_keeps_existing_ordinary_ui_path():
    assert should_send_sort_through_ordinary_ui(_Track(), set()) is True


def test_rid_owned_sort_is_excluded_from_visual_ranging_candidates():
    owned = _Track(sort_id=238, created_ts=100.5)
    ordinary = _Track(sort_id=239, created_ts=100.6)

    selected = tracking.select_ordinary_ui_tracks_for_output(
        [owned, ordinary],
        owned_special_generations={SortGeneration(238, 100.5)},
        special_rid_ui_exclusive=False,
    )

    assert selected == [ordinary]


def test_ordinary_ui_allocator_skips_reserved_special_ids():
    assert tracking.next_unreserved_ui_id(1, frozenset((1, 2))) == 3
    assert tracking.next_unreserved_ui_id(3, frozenset((1, 2))) == 3


def test_ordinary_visual_ui_ids_cycle_between_three_and_five():
    allocator = tracking.CyclicUiIdAllocator((3, 4, 5))

    assert allocator.get_or_assign(101) == 3
    assert allocator.get_or_assign(102) == 4
    assert allocator.get_or_assign(103) == 5
    assert allocator.get_or_assign(104) is None

    allocator.release_inactive((102, 103))
    assert allocator.get_or_assign(104) == 3
    assert allocator.get_or_assign(105) is None

    allocator.release(102)
    assert allocator.get_or_assign(105) == 4


def test_ordinary_visual_ui_id_stays_stable_while_track_is_active():
    allocator = tracking.CyclicUiIdAllocator((3, 4, 5))

    assert allocator.get_or_assign(101) == 3
    assert allocator.get_or_assign(101) == 3


def test_rid_takeover_keeps_old_visual_id_reserved_until_delete_is_acked():
    allocator = tracking.CyclicUiIdAllocator((3, 4, 5))
    tracks = [_Track(sort_id=101), _Track(sort_id=102), _Track(sort_id=103)]
    for track, expected in zip(tracks, (3, 4, 5)):
        assert allocator.get_or_assign(track.id) == expected

    class _Registry:
        pending = True

        def owner_of(self, generation):
            return object() if generation.sort_id == 101 else None

        def replacement_pending_for_generation(self, generation):
            return self.pending and generation.sort_id == 101

    registry = _Registry()
    tracking.release_completed_special_ui_assignments(
        allocator, tracks, registry
    )
    assert allocator.get_or_assign(104) is None

    registry.pending = False
    tracking.release_completed_special_ui_assignments(
        allocator, tracks, registry
    )
    assert allocator.get_or_assign(104) == 3


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


def test_special_rid_sort_freshness_defaults_to_six_seconds():
    assert tracking.SPECIAL_RID_SORT_FRESH_SECONDS == 6.0


def test_sort_reacquire_gate_defaults_to_five_degrees():
    assert tracking.TRACK_REACQUIRE_MAX_DEG == 5.0


def test_strike_candidates_require_exact_special_sort_generation():
    owned = _Track(sort_id=238, created_ts=100.5)
    reused_sort_id = _Track(sort_id=238, created_ts=200.5)
    unowned = _Track(sort_id=900, created_ts=201.0)
    registry = _SpecialRidOwnerRegistry({SortGeneration(238, 100.5): 1})

    selected = tracking.select_special_rid_strike_tracks(
        [owned, reused_sort_id, unowned],
        registry,
    )

    assert selected == [owned]
    assert tracking.special_rid_ui_id_for_track(owned, registry) == 1
    assert tracking.special_rid_ui_id_for_track(reused_sort_id, registry) is None


def test_strike_candidates_reject_owner_ids_outside_one_and_two():
    track = _Track(sort_id=238, created_ts=100.5)
    registry = _SpecialRidOwnerRegistry({SortGeneration(238, 100.5): 3})

    assert tracking.select_special_rid_strike_tracks([track], registry) == []
    assert tracking.special_rid_ui_id_for_track(track, registry) is None


def test_visual_strike_candidates_use_only_locked_unowned_sort():
    special = _Track(sort_id=238, created_ts=100.5)
    locked_visual = _Track(sort_id=239, created_ts=100.6)
    other_visual = _Track(sort_id=240, created_ts=100.7)
    registry = _SpecialRidOwnerRegistry({SortGeneration(238, 100.5): 1})

    selected = tracking.select_visual_strike_tracks(
        [special, locked_visual, other_visual],
        master_id=239,
        special_rid_registry=registry,
        special_rid_ui_exclusive=False,
    )

    assert selected == [locked_visual]


def test_visual_strike_candidates_are_disabled_by_exclusive_mode():
    locked_visual = _Track(sort_id=239, created_ts=100.6)

    selected = tracking.select_visual_strike_tracks(
        [locked_visual],
        master_id=239,
        special_rid_registry=_SpecialRidOwnerRegistry({}),
        special_rid_ui_exclusive=True,
    )

    assert selected == []


def test_ui_and_strike_tracks_become_confirmed_on_sixteenth_hit():
    track = _Track()
    track.hit_streak = 15
    track.ui_confirmed = False
    track.strike_confirmed = False

    tracking.update_track_confirmation_flags(track)
    assert track.ui_confirmed is False
    assert track.strike_confirmed is False

    track.hit_streak = 16
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


def test_track_conversion_carries_previously_sent_visual_ui_id():
    observation = sort_observation_from_track(
        _Track(),
        map_azimuth=lambda relative: relative,
        replaced_visual_ui_id=5,
    )

    assert observation.replaced_visual_ui_id == 5


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
        replaced_target_id=3,
    )

    assert len(fake_socket.packet) == 34
    assert struct.unpack("!BB8sIffffI", fake_socket.packet)[8] == 3


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


def test_gps_sender_stores_station_altitude_in_rid_ellipsoid_reference(
    monkeypatch,
):
    class _StopAfterFirstCycle(Exception):
        pass

    class _PositionState:
        altitude = None

        def update(self, *, longitude, latitude, altitude, source):
            self.altitude = altitude

    class _Sender:
        def send_gps_location(self, *, latitude, longitude):
            return True

    def fake_read_gps_fix(*, altitude_reference, **_kwargs):
        altitude = 831.3138 if altitude_reference == "ellipsoid" else 862.002
        return 107.10433499, 27.95072567, altitude, "test-gps"

    def stop_after_first_cycle(_seconds):
        raise _StopAfterFirstCycle

    position_state = _PositionState()
    monkeypatch.setattr(tracking, "read_gps_fix", fake_read_gps_fix)
    monkeypatch.setattr(tracking, "field_log_event", lambda _event: None)
    monkeypatch.setattr(tracking.time, "sleep", stop_after_first_cycle)

    with pytest.raises(_StopAfterFirstCycle):
        tracking.gps_sender_thread(_Sender(), position_state)

    assert position_state.altitude == pytest.approx(831.3138)
