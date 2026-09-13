import math
import sys
from pathlib import Path

import pytest


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

from special_rid_identity import (  # noqa: E402
    ST22Q_RID,
    XDB_RID,
    SortGeneration,
    SortObservation,
    SpecialRidPredictor,
    SpecialRidRegistry,
    run_special_rid_ui_sender,
)


EARTH_RADIUS_M = 6_371_008.8


def _longitude_offset_m(east_m, latitude=30.0, base_longitude=104.0):
    return base_longitude + math.degrees(
        float(east_m) / (EARTH_RADIUS_M * math.cos(math.radians(latitude)))
    )


def _rid_snapshot(
    rid_id="RID-A",
    *,
    timestamp=10.0,
    measurement_seq=1,
    east_m=0.0,
    speed=10.0,
    heading=90.0,
    vertical_speed=0.0,
):
    sample = {
        "measurement_seq": measurement_seq,
        "timestamp": timestamp,
        "longitude": _longitude_offset_m(east_m),
        "latitude": 30.0,
        "alt_geo": 500.0,
        "horizontal_speed": speed,
        "track_heading": heading,
    }
    return {
        "key": ("GB42590-2023", 1, rid_id),
        "rid_id": rid_id,
        "position_valid": True,
        "last_receive_ts": timestamp,
        "measurement_seq": measurement_seq,
        "measurement_count": measurement_seq,
        "measurement_history": [sample],
        "horizontal_speed": speed,
        "vertical_speed": vertical_speed,
        "track_heading": heading,
    }


def test_predictor_advances_at_current_time_without_display_delay():
    predictor = SpecialRidPredictor(max_prediction_s=5.0, fresh_s=7.0)
    assert predictor.observe(_rid_snapshot())

    point = predictor.predict(11.0)

    assert point.mode == "prediction"
    assert point.prediction_age_s == pytest.approx(1.0)
    assert point.east_m == pytest.approx(predictor.measured_east_m + 10.0, abs=0.2)


def test_predictor_freezes_after_five_seconds_and_expires_after_seven():
    predictor = SpecialRidPredictor(max_prediction_s=5.0, fresh_s=7.0)
    predictor.observe(_rid_snapshot())

    at_six = predictor.predict(16.0)

    assert at_six.mode == "freeze"
    assert at_six.east_m == pytest.approx(predictor.measured_east_m + 50.0, abs=0.2)
    assert predictor.predict(17.0) is not None
    assert predictor.predict(17.001) is None


def test_prediction_points_do_not_feed_back_into_measurement_state():
    predictor = SpecialRidPredictor(max_prediction_s=5.0, fresh_s=7.0)
    predictor.observe(_rid_snapshot())

    predictor.predict(10.2)
    predictor.predict(10.4)

    assert predictor.measurement_count == 1


def test_duplicate_packet_refreshes_receive_age_without_becoming_measurement():
    predictor = SpecialRidPredictor(max_prediction_s=5.0, fresh_s=7.0)
    first = _rid_snapshot()
    predictor.observe(first)
    duplicate = dict(first)
    duplicate["last_receive_ts"] = 12.0

    assert predictor.observe(duplicate) is False
    assert predictor.measurement_count == 1
    assert predictor.predict(18.0).receive_age_s == pytest.approx(6.0)


def test_latest_top_level_vertical_speed_is_used_when_compiled_history_omits_it():
    predictor = SpecialRidPredictor(max_prediction_s=5.0, fresh_s=7.0)
    predictor.observe(_rid_snapshot(vertical_speed=2.5))

    point = predictor.predict(12.0)

    assert point.altitude_m == pytest.approx(505.0, abs=0.2)


def _station():
    return {
        "valid": True,
        "latitude": 30.0,
        "longitude": 104.0,
        "altitude": 450.0,
    }


def _rid_at_angles(
    rid_id, azimuth, elevation, timestamp, *, measurement_seq=1
):
    horizontal_m = 100.0
    azimuth_rad = math.radians(azimuth)
    east_m = horizontal_m * math.sin(azimuth_rad)
    north_m = horizontal_m * math.cos(azimuth_rad)
    snapshot = _rid_snapshot(
        rid_id,
        timestamp=timestamp,
        measurement_seq=measurement_seq,
        east_m=east_m,
        speed=0.0,
        heading=0.0,
    )
    sample = snapshot["measurement_history"][0]
    sample["latitude"] = 30.0 + math.degrees(north_m / EARTH_RADIUS_M)
    sample["alt_geo"] = 450.0 + math.tan(math.radians(elevation)) * horizontal_m
    return snapshot


def _sort_observation(
    sort_id,
    *,
    azimuth,
    elevation,
    created_ts,
    ui_confirmed=True,
    logic_id=13,
    board="BOARD_3",
    camera_id=2,
    last_detection_ts=None,
    hit_streak=20,
    velocity_az=0.0,
    velocity_el=0.0,
):
    return SortObservation(
        generation=SortGeneration(sort_id, created_ts),
        ui_confirmed=ui_confirmed,
        map_az=azimuth,
        elevation=elevation,
        velocity_az=velocity_az,
        velocity_el=velocity_el,
        board=board,
        camera_id=camera_id,
        logic_id=logic_id,
        last_detection_ts=(
            created_ts if last_detection_ts is None else last_detection_ts
        ),
        hit_streak=hit_streak,
    )


def test_first_successfully_bound_rid_gets_one_even_when_it_is_st22q():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 20.0, 9.0, 10.0)], now_ts=10.0
    )

    registry.observe_sorts(
        [_sort_observation(7, azimuth=20.2, elevation=9.1, created_ts=10.5)],
        _station(),
        now_ts=11.0,
    )

    assert registry.slot_for_rid(ST22Q_RID).ui_id == 1


def test_confirmed_but_not_ui_confirmed_sort_cannot_register():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 15.0, 3.0, 10.0)], now_ts=10.0
    )

    registry.observe_sorts(
        [
            _sort_observation(
                8,
                azimuth=15.0,
                elevation=3.0,
                created_ts=10.5,
                ui_confirmed=False,
            )
        ],
        _station(),
        now_ts=11.0,
    )

    assert registry.slot_for_rid(XDB_RID).ui_id is None


def test_two_rids_receive_one_and_two_in_successful_binding_order():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 15.0, 3.0, 10.0)], now_ts=10.0
    )
    registry.observe_sorts(
        [_sort_observation(177, azimuth=15.0, elevation=3.0, created_ts=10.5)],
        _station(),
        now_ts=11.0,
    )
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 30.0, 8.0, 20.0)], now_ts=20.0
    )
    registry.observe_sorts(
        [_sort_observation(189, azimuth=30.0, elevation=8.0, created_ts=20.5)],
        _station(),
        now_ts=21.0,
    )

    assert registry.slot_for_rid(XDB_RID).ui_id == 1
    assert registry.slot_for_rid(ST22Q_RID).ui_id == 2


def test_initial_binding_uses_wrapped_azimuth_plus_elevation_distance():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 359.0, 10.0, 10.0)], now_ts=10.0
    )

    registry.observe_sorts(
        [
            _sort_observation(1, azimuth=1.0, elevation=10.0, created_ts=10.5),
            _sort_observation(2, azimuth=350.0, elevation=25.0, created_ts=10.6),
        ],
        _station(),
        now_ts=11.0,
    )

    assert registry.slot_for_rid(XDB_RID).current_sort.sort_id == 1


def test_non_whitelisted_rid_and_preexisting_sort_are_not_registered():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [_rid_at_angles("RID-OTHER", 15.0, 3.0, 10.0)], now_ts=10.0
    )
    assert registry.slot_for_rid("RID-OTHER") is None

    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 15.0, 3.0, 10.0)], now_ts=10.0
    )
    registry.observe_sorts(
        [_sort_observation(9, azimuth=15.0, elevation=3.0, created_ts=9.0)],
        _station(),
        now_ts=11.0,
    )
    assert registry.slot_for_rid(XDB_RID).ui_id is None


def test_registration_ignores_old_candidate_and_still_binds_available_new_sort():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [
            _rid_at_angles(XDB_RID, 15.0, 3.0, 10.0),
            _rid_at_angles(ST22Q_RID, 30.0, 8.0, 10.0),
        ],
        now_ts=10.0,
    )

    registry.observe_sorts(
        [
            _sort_observation(90, azimuth=15.0, elevation=3.0, created_ts=9.0),
            _sort_observation(91, azimuth=30.0, elevation=8.0, created_ts=10.5),
        ],
        _station(),
        now_ts=11.0,
    )

    registered = registry.registered_slots()
    assert len(registered) == 1
    assert registered[0].rid_id == ST22Q_RID
    assert registered[0].current_sort.sort_id == 91


def test_preexisting_sort_cannot_be_absorbed_as_registered_slot_successor():
    registry = SpecialRidRegistry()
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 15.0, 3.0, 10.0)], now_ts=10.0
    )

    registry.observe_sorts(
        [
            _sort_observation(
                90,
                azimuth=15.2,
                elevation=3.0,
                created_ts=9.0,
                last_detection_ts=11.0,
            ),
            _sort_observation(
                91,
                azimuth=15.0,
                elevation=3.0,
                created_ts=10.5,
                last_detection_ts=11.0,
            ),
        ],
        _station(),
        now_ts=11.0,
    )

    assert registry.slot_for_rid(XDB_RID).current_sort.sort_id == 91
    assert registry.owner_of_sort_id(90) is None


def _registry_with_two_registered_slots():
    registry = SpecialRidRegistry(sort_fresh_s=4.0, reacquire_delay_s=0.0)
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 15.0, 3.0, 1.0)], now_ts=1.0
    )
    registry.observe_sorts(
        [_sort_observation(177, azimuth=15.0, elevation=3.0, created_ts=1.1)],
        _station(),
        now_ts=1.2,
    )
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 2.0)], now_ts=2.0
    )
    registry.observe_sorts(
        [_sort_observation(189, azimuth=100.0, elevation=10.0, created_ts=2.1)],
        _station(),
        now_ts=2.2,
    )
    return registry


def test_log_chain_189_196_202_205_238_becomes_one_family():
    registry = _registry_with_two_registered_slots()

    registry.observe_rids(
        [
            _rid_at_angles(
                ST22Q_RID,
                101.49,
                10.0,
                34.7,
                measurement_seq=2,
            )
        ],
        now_ts=34.7,
    )
    registry.observe_sorts(
        [
            _sort_observation(
                196,
                azimuth=101.49,
                elevation=10.0,
                created_ts=34.7,
                logic_id=13,
                last_detection_ts=34.7,
            )
        ],
        _station(),
        now_ts=34.7,
    )
    registry.observe_sorts(
        [
            _sort_observation(
                196,
                azimuth=102.0,
                elevation=10.0,
                created_ts=34.7,
                logic_id=12,
                camera_id=1,
                last_detection_ts=35.0,
            ),
            _sort_observation(
                202,
                azimuth=109.063,
                elevation=10.0,
                created_ts=37.3,
                logic_id=12,
                camera_id=1,
                last_detection_ts=37.3,
            ),
        ],
        _station(),
        now_ts=37.3,
    )
    assert registry.owner_of_sort_id(202) is None

    registry.observe_sorts(
        [
            _sort_observation(
                196,
                azimuth=106.0,
                elevation=10.0,
                created_ts=34.7,
                logic_id=12,
                camera_id=1,
                last_detection_ts=39.0,
            ),
            _sort_observation(
                202,
                azimuth=106.93,
                elevation=10.0,
                created_ts=37.3,
                logic_id=12,
                camera_id=1,
                last_detection_ts=39.0,
            ),
        ],
        _station(),
        now_ts=39.0,
    )
    registry.observe_rids(
        [
            _rid_at_angles(
                ST22Q_RID,
                114.2,
                10.0,
                44.5,
                measurement_seq=3,
            )
        ],
        now_ts=44.5,
    )
    registry.observe_sorts(
        [
            _sort_observation(
                205,
                azimuth=114.198,
                elevation=10.0,
                created_ts=40.1,
                logic_id=13,
                camera_id=2,
                last_detection_ts=44.5,
            )
        ],
        _station(),
        now_ts=44.5,
    )
    registry.observe_sorts(
        [
            _sort_observation(
                205,
                azimuth=120.0,
                elevation=10.0,
                created_ts=40.1,
                logic_id=13,
                camera_id=2,
                last_detection_ts=70.0,
            ),
            _sort_observation(
                238,
                azimuth=120.41,
                elevation=10.0,
                created_ts=70.2,
                logic_id=13,
                camera_id=2,
                last_detection_ts=70.2,
            ),
        ],
        _station(),
        now_ts=70.2,
    )

    family_ids = {
        generation.sort_id
        for generation in registry.slot_for_rid(ST22Q_RID).sort_family
    }
    assert {189, 196, 202, 205, 238} <= family_ids


def test_fresh_slot_cannot_absorb_normal_successor_needed_by_missing_slot():
    registry = SpecialRidRegistry(sort_fresh_s=4.0)
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 14.0, 4.0, 1.0)], now_ts=1.0
    )
    registry.observe_sorts(
        [_sort_observation(177, azimuth=14.0, elevation=4.0, created_ts=1.1)],
        _station(),
        now_ts=1.2,
    )
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 14.0, 10.0, 2.0)], now_ts=2.0
    )
    registry.observe_sorts(
        [_sort_observation(189, azimuth=14.0, elevation=10.0, created_ts=2.1)],
        _station(),
        now_ts=2.2,
    )
    registry.observe_rids(
        [
            _rid_at_angles(
                ST22Q_RID,
                14.0,
                10.0,
                10.0,
                measurement_seq=2,
            )
        ],
        now_ts=10.0,
    )
    registry.observe_sorts(
        [
            _sort_observation(
                177,
                azimuth=14.0,
                elevation=4.0,
                created_ts=1.1,
                last_detection_ts=10.0,
            ),
            _sort_observation(
                196,
                azimuth=14.0,
                elevation=6.0,
                created_ts=9.5,
                last_detection_ts=10.0,
            ),
        ],
        _station(),
        now_ts=10.0,
    )

    assert registry.owner_of_sort_id(196).rid_id == ST22Q_RID


def test_stale_unowned_sort_is_not_attached_after_current_source_expires():
    registry = SpecialRidRegistry(sort_fresh_s=4.0)
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 1.0)], now_ts=1.0
    )
    registry.observe_sorts(
        [_sort_observation(189, azimuth=100.0, elevation=10.0, created_ts=1.1)],
        _station(),
        now_ts=1.2,
    )

    registry.observe_sorts(
        [
            _sort_observation(
                196,
                azimuth=101.0,
                elevation=10.0,
                created_ts=2.0,
                last_detection_ts=2.0,
            )
        ],
        _station(),
        now_ts=10.0,
    )

    assert registry.owner_of_sort_id(196) is None


def test_owned_sort_238_cannot_be_stolen_by_xdb_after_forced_timeout():
    registry = _registry_with_two_registered_slots()
    registry.observe_sorts(
        [
            _sort_observation(
                238,
                azimuth=100.5,
                elevation=10.0,
                created_ts=2.5,
                last_detection_ts=2.5,
            )
        ],
        _station(),
        now_ts=2.5,
    )
    assert registry.owner_of_sort_id(238).rid_id == ST22Q_RID

    registry.observe_sorts(
        [
            _sort_observation(
                238,
                azimuth=15.0,
                elevation=3.0,
                created_ts=2.5,
                last_detection_ts=60.0,
            )
        ],
        _station(),
        now_ts=60.0,
    )

    assert registry.owner_of_sort_id(238).rid_id == ST22Q_RID
    assert registry.ownership_reject_count >= 1


def test_current_sort_is_sticky_for_four_seconds_then_switches():
    registry = SpecialRidRegistry(sort_fresh_s=4.0)
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 19.0)], now_ts=19.0
    )
    registry.observe_sorts(
        [
            _sort_observation(
                196,
                azimuth=100.0,
                elevation=10.0,
                created_ts=19.5,
                last_detection_ts=20.0,
            )
        ],
        _station(),
        now_ts=20.0,
    )
    registry.observe_sorts(
        [
            _sort_observation(
                202,
                azimuth=100.5,
                elevation=10.0,
                created_ts=21.0,
                last_detection_ts=22.0,
            )
        ],
        _station(),
        now_ts=22.0,
    )

    registry.select_current_sorts(now_ts=23.9)
    assert registry.slot_for_rid(ST22Q_RID).current_sort.sort_id == 196
    registry.select_current_sorts(now_ts=24.1)
    assert registry.slot_for_rid(ST22Q_RID).current_sort.sort_id == 202


def test_stale_current_sort_immediately_reacquires_unique_far_sort():
    registry = SpecialRidRegistry(sort_fresh_s=4.0, reacquire_delay_s=0.0)
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 9.0)], now_ts=9.0
    )
    registry.observe_sorts(
        [
            _sort_observation(
                189,
                azimuth=250.0,
                elevation=40.0,
                created_ts=9.5,
                last_detection_ts=10.0,
            )
        ],
        _station(),
        now_ts=10.0,
    )

    registry.observe_sorts(
        [
            _sort_observation(
                900,
                azimuth=150.0,
                elevation=40.0,
                created_ts=14.1,
                last_detection_ts=14.1,
            )
        ],
        _station(),
        now_ts=14.1,
    )

    assert registry.owner_of_sort_id(900).rid_id == ST22Q_RID
    assert registry.slot_for_rid(ST22Q_RID).current_sort.sort_id == 900


def test_stale_reacquire_uses_current_rid_not_old_false_sort_geometry():
    registry = SpecialRidRegistry(sort_fresh_s=4.0, reacquire_delay_s=0.0)
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 9.0)], now_ts=9.0
    )
    registry.observe_sorts(
        [
            _sort_observation(
                189,
                azimuth=250.0,
                elevation=40.0,
                created_ts=9.5,
                last_detection_ts=10.0,
            )
        ],
        _station(),
        now_ts=10.0,
    )

    registry.observe_sorts(
        [
            _sort_observation(
                900,
                azimuth=100.5,
                elevation=10.2,
                created_ts=14.1,
                last_detection_ts=14.1,
            ),
            _sort_observation(
                901,
                azimuth=250.5,
                elevation=40.1,
                created_ts=14.2,
                last_detection_ts=14.2,
            ),
        ],
        _station(),
        now_ts=14.2,
    )

    assert registry.owner_of_sort_id(900).rid_id == ST22Q_RID
    assert registry.owner_of_sort_id(901) is None
    assert registry.slot_for_rid(ST22Q_RID).current_sort.sort_id == 900


def test_fresh_current_sort_keeps_far_unowned_candidate_unassigned():
    registry = SpecialRidRegistry(sort_fresh_s=4.0, reacquire_delay_s=0.0)
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 9.0)], now_ts=9.0
    )
    registry.observe_sorts(
        [
            _sort_observation(
                189,
                azimuth=150.0,
                elevation=40.0,
                created_ts=9.5,
                last_detection_ts=10.0,
            )
        ],
        _station(),
        now_ts=10.0,
    )

    registry.observe_sorts(
        [
            _sort_observation(
                900,
                azimuth=100.0,
                elevation=10.0,
                created_ts=12.0,
                last_detection_ts=13.9,
            )
        ],
        _station(),
        now_ts=13.9,
    )

    assert registry.owner_of_sort_id(900) is None
    assert registry.slot_for_rid(ST22Q_RID).current_sort.sort_id == 189


def test_immediate_reacquire_keeps_rid_ui_status_available_in_same_cycle():
    registry = SpecialRidRegistry(sort_fresh_s=4.0, reacquire_delay_s=0.0)
    registry.observe_rids(
        [_rid_at_angles(XDB_RID, 100.0, 10.0, 9.0)], now_ts=9.0
    )
    registry.observe_sorts(
        [
            _sort_observation(
                189,
                azimuth=250.0,
                elevation=40.0,
                created_ts=9.5,
                last_detection_ts=10.0,
                board="BOARD_3",
                camera_id=2,
            )
        ],
        _station(),
        now_ts=10.0,
    )

    registry.observe_sorts(
        [
            _sort_observation(
                900,
                azimuth=150.0,
                elevation=40.0,
                created_ts=14.1,
                last_detection_ts=14.1,
                board="BOARD_4",
                camera_id=3,
            )
        ],
        _station(),
        now_ts=14.1,
    )
    statuses = registry.ui_statuses(now_ts=14.1, station=_station())

    assert len(statuses) == 1
    assert statuses[0].target_id == 1
    assert statuses[0].azimuth == pytest.approx(100.0, abs=0.02)
    assert (statuses[0].board, statuses[0].camera_id) == ("BOARD_4", 3)


def test_configured_reacquire_delay_starts_after_sort_freshness_expires():
    registry = SpecialRidRegistry(sort_fresh_s=4.0, reacquire_delay_s=2.0)
    registry.observe_rids(
        [_rid_at_angles(ST22Q_RID, 100.0, 10.0, 9.0)], now_ts=9.0
    )
    registry.observe_sorts(
        [
            _sort_observation(
                189,
                azimuth=250.0,
                elevation=40.0,
                created_ts=9.5,
                last_detection_ts=10.0,
            )
        ],
        _station(),
        now_ts=10.0,
    )

    registry.observe_sorts(
        [
            _sort_observation(
                900,
                azimuth=150.0,
                elevation=40.0,
                created_ts=14.1,
                last_detection_ts=14.1,
            )
        ],
        _station(),
        now_ts=14.1,
    )
    assert registry.owner_of_sort_id(900) is None

    registry.observe_sorts(
        [
            _sort_observation(
                900,
                azimuth=150.0,
                elevation=40.0,
                created_ts=14.1,
                last_detection_ts=16.0,
            )
        ],
        _station(),
        now_ts=16.0,
    )

    assert registry.owner_of_sort_id(900).rid_id == ST22Q_RID
    assert registry.slot_for_rid(ST22Q_RID).current_sort.sort_id == 900


def _ready_registry(
    *,
    now_ts=20.0,
    rid_age=0.0,
    sort_age=0.0,
    rid_az=170.0,
    rid_el=9.0,
    sort_az=190.0,
    sort_el=30.0,
):
    registry = SpecialRidRegistry(sort_fresh_s=4.0)
    rid_ts = now_ts - rid_age
    rid_item = _rid_at_angles(XDB_RID, rid_az, rid_el, rid_ts)
    registry.observe_rids([rid_item], now_ts=rid_ts)
    sort_last_ts = now_ts - sort_age
    sort_created_ts = max(rid_ts + 0.001, sort_last_ts - 0.1)
    registry.observe_sorts(
        [
            _sort_observation(
                700,
                azimuth=sort_az,
                elevation=sort_el,
                created_ts=sort_created_ts,
                last_detection_ts=sort_last_ts,
                board="BOARD_3",
                camera_id=2,
            )
        ],
        _station(),
        now_ts=sort_last_ts,
    )
    return registry, rid_item


def test_ui_status_uses_rid_geometry_and_sort_source_only():
    registry, _ = _ready_registry()

    statuses = registry.ui_statuses(now_ts=20.0, station=_station())

    assert len(statuses) == 1
    status = statuses[0]
    assert status.azimuth == pytest.approx(170.0, abs=0.02)
    assert status.elevation == pytest.approx(9.0, abs=0.02)
    assert status.distance == pytest.approx(
        math.hypot(100.0, math.tan(math.radians(9.0)) * 100.0),
        abs=0.2,
    )
    assert (status.board, status.camera_id) == ("BOARD_3", 2)
    assert status.target_id == 1
    assert status.replaced_target_id == 0


def test_special_threat_boundary_matches_existing_ui_rule():
    assert SpecialRidRegistry._threat_score(99.999) == 100.0
    assert SpecialRidRegistry._threat_score(100.0) == 50.0
    assert SpecialRidRegistry._threat_score(300.0) == 50.0
    assert SpecialRidRegistry._threat_score(300.001) == 0.0


@pytest.mark.parametrize(
    "rid_age,sort_age,expected",
    [
        (7.0, 4.0, 1),
        (7.001, 4.0, 0),
        (7.0, 4.001, 0),
    ],
)
def test_special_ui_uses_independent_seven_and_four_second_gates(
    rid_age, sort_age, expected
):
    registry, _ = _ready_registry(rid_age=rid_age, sort_age=sort_age)

    assert len(registry.ui_statuses(now_ts=20.0, station=_station())) == expected


class _RecordingSender:
    def __init__(self, clock):
        self.clock = clock
        self.calls = []

    def send_status(self, **status):
        self.calls.append((self.clock(), status))
        return True


class _OverrunStopEvent:
    def __init__(self):
        self.wall = 0.1
        self.monotonic_time = 0.0
        self.wait_count = 0
        self.stopped = False

    def is_set(self):
        return self.stopped

    def clock(self):
        return self.wall

    def monotonic(self):
        return self.monotonic_time

    def wait(self, timeout):
        self.wait_count += 1
        if self.wait_count == 1:
            self.wall = 1.1
            self.monotonic_time = 1.0
            return False
        if self.wait_count == 2:
            assert timeout == pytest.approx(0.2)
            self.wall = 1.3
            self.monotonic_time = 1.2
            return False
        self.stopped = True
        return True


def test_sender_thread_skips_missed_deadlines_instead_of_catch_up_burst():
    registry, rid_item = _ready_registry(now_ts=0.1, rid_az=20.0, rid_el=5.0)
    scheduler = _OverrunStopEvent()
    sender = _RecordingSender(scheduler.clock)

    run_special_rid_ui_sender(
        registry=registry,
        rid_snapshot_provider=lambda: [rid_item],
        station_snapshot_provider=_station,
        sender=sender,
        stop_event=scheduler,
        hz=5.0,
        clock=scheduler.clock,
        monotonic=scheduler.monotonic,
    )

    assert [round(timestamp, 1) for timestamp, _ in sender.calls] == [0.1, 1.1, 1.3]
