#!/usr/bin/env python3
"""Replay the 20260912 special-RID SORT lineage through production logic."""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
CORE_DIR = ROOT_DIR / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from special_rid_identity import (  # noqa: E402
    SPECIAL_RID_IDS,
    ST22Q_RID,
    XDB_RID,
    SortGeneration,
    SortObservation,
    SpecialRidRegistry,
)


REPLAY_SORT_IDS = frozenset((177, 189, 196, 202, 205, 238))
_STATION_PATTERN = re.compile(
    r"lat=(?P<lat>-?[0-9.]+),lon=(?P<lon>-?[0-9.]+),"
    r"alt_msl_m=(?P<alt>-?[0-9.]+)"
)


@dataclass(frozen=True)
class ReplayResult:
    registration_order: list
    ui_ids: dict
    sort_families: dict
    owner_by_sort: dict
    current_sort_by_rid: dict
    forbidden_steal_count: int

    def json_ready(self):
        payload = asdict(self)
        payload["sort_families"] = {
            rid_id: sorted(generations)
            for rid_id, generations in payload["sort_families"].items()
        }
        return payload


def _first_path(log_dir, pattern):
    paths = sorted(Path(log_dir).glob(pattern))
    if not paths:
        raise FileNotFoundError(f"missing {pattern} in {log_dir}")
    return paths[0]


def _float(value, default=None):
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return default
    return parsed if math.isfinite(parsed) else default


def _int(value, default=None):
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return default


def _load_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def _station_from_events(rows):
    for row in rows:
        if row.get("event") != "GPS_STATION_FIX":
            continue
        match = _STATION_PATTERN.search(str(row.get("reason", "")))
        if match:
            return {
                "valid": True,
                "latitude": float(match.group("lat")),
                "longitude": float(match.group("lon")),
                "altitude": float(match.group("alt")),
            }
    raise ValueError("GPS_STATION_FIX with lat/lon/alt_msl_m was not found")


class _RidSnapshots:
    def __init__(self):
        self._states = {}

    def ingest(self, record):
        receive_ts = _float(record.get("receive_ts"))
        payload = record.get("payload") or {}
        uav = payload.get("UAVInfo") or {}
        rid_id = str(uav.get("ID", ""))
        if receive_ts is None or rid_id not in SPECIAL_RID_IDS:
            return
        latitude = _float(uav.get("Lat"))
        longitude = _float(uav.get("Lon"))
        if latitude is None or longitude is None or (latitude == 0.0 and longitude == 0.0):
            return
        state = self._states.setdefault(rid_id, {
            "sequence": 0,
            "signature": None,
            "history": [],
        })
        signature = (
            uav.get("T_Stamp"),
            latitude,
            longitude,
            _float(uav.get("AltGeo")),
            _float(uav.get("H_Speed")),
            _float(uav.get("V_Speed")),
            _float(uav.get("Trk")),
        )
        if signature != state["signature"]:
            state["sequence"] += 1
            state["signature"] = signature
            state["history"].append({
                "measurement_seq": state["sequence"],
                "timestamp": receive_ts,
                "latitude": latitude,
                "longitude": longitude,
                "alt_geo": _float(uav.get("AltGeo")),
                "horizontal_speed": _float(uav.get("H_Speed")),
                "vertical_speed": _float(uav.get("V_Speed")),
                "track_heading": _float(uav.get("Trk")),
            })
            del state["history"][:-64]
        state.update({
            "last_receive_ts": receive_ts,
            "latitude": latitude,
            "longitude": longitude,
            "alt_geo": _float(uav.get("AltGeo")),
            "horizontal_speed": _float(uav.get("H_Speed")),
            "vertical_speed": _float(uav.get("V_Speed")),
            "track_heading": _float(uav.get("Trk")),
            "standard": str(uav.get("RID_Standard", "")),
            "id_type": _int(uav.get("ID_Type"), 0),
        })

    def snapshots(self):
        result = []
        for rid_id, state in self._states.items():
            result.append({
                "key": (state["standard"], state["id_type"], rid_id),
                "rid_id": rid_id,
                "position_valid": True,
                "last_receive_ts": state["last_receive_ts"],
                "measurement_seq": state["sequence"],
                "measurement_count": state["sequence"],
                "measurement_history": [dict(item) for item in state["history"]],
                "latitude": state["latitude"],
                "longitude": state["longitude"],
                "alt_geo": state["alt_geo"],
                "horizontal_speed": state["horizontal_speed"],
                "vertical_speed": state["vertical_speed"],
                "track_heading": state["track_heading"],
            })
        return result


class _MeasurementIndex:
    def __init__(self, rows):
        self.by_index = {}
        for row in rows:
            timestamp = _float(row.get("timestamp"))
            measurement_index = _int(row.get("meas_idx"))
            if timestamp is None or measurement_index is None:
                continue
            self.by_index.setdefault(measurement_index, []).append((timestamp, row))
        for values in self.by_index.values():
            values.sort(key=lambda item: item[0])

    def nearest(self, measurement_index, timestamp, tolerance=0.05):
        values = self.by_index.get(measurement_index, ())
        if not values:
            return None
        times = [item[0] for item in values]
        position = bisect.bisect_left(times, timestamp)
        candidates = values[max(0, position - 1): position + 2]
        if not candidates:
            return None
        best = min(candidates, key=lambda item: abs(item[0] - timestamp))
        return best[1] if abs(best[0] - timestamp) <= tolerance else None


def _load_rid_records(path):
    records = []
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if _float(record.get("receive_ts")) is not None:
                records.append(record)
    records.sort(key=lambda item: float(item["receive_ts"]))
    return records


def replay_run(log_dir, start_ts=1789156230.355754, device_heading_deg=180.0):
    log_dir = Path(log_dir)
    event_rows = _load_csv(_first_path(log_dir, "events_*.csv"))
    measurement_rows = _load_csv(_first_path(log_dir, "measurements_*.csv"))
    rid_records = _load_rid_records(_first_path(log_dir, "raw_rid_*.jsonl"))
    station = _station_from_events(event_rows)
    measurements = _MeasurementIndex(measurement_rows)

    created_by_track = {}
    replay_events = []
    for row in event_rows:
        track_id = _int(row.get("track_id"))
        timestamp = _float(row.get("timestamp"))
        if track_id not in REPLAY_SORT_IDS or timestamp is None:
            continue
        if row.get("event") == "NEW_TRACK":
            created_by_track[track_id] = timestamp
        if (
            timestamp >= float(start_ts)
            and row.get("event") in ("NEW_TRACK", "MATCH_ACCEPT")
            and _float(row.get("meas_az")) is not None
            and _float(row.get("meas_el")) is not None
        ):
            replay_events.append((timestamp, row))
    replay_events.sort(key=lambda item: item[0])
    seed = next(
        ((timestamp, row) for timestamp, row in replay_events if _int(row.get("track_id")) == 177),
        None,
    )
    if seed is None:
        raise ValueError("SORT177 checkpoint observation was not found")
    # The requested replay begins with XDB already bound to the old UI28 / SORT177.
    # Rebase that existing generation onto the checkpoint so the new runtime can
    # seed UI1 without pretending the pre-checkpoint registration happened again.
    created_by_track[177] = float(start_ts)

    registration_order = []

    def capture_event(row):
        if row.get("event") == "REGISTERED":
            registration_order.append(str(row.get("rid_id")))

    registry = SpecialRidRegistry(
        log_callback=capture_event,
        sort_fresh_s=4.0,
        sort_internal_s=12.0,
        reacquire_delay_s=0.0,
    )
    rid_state = _RidSnapshots()
    rid_position = 0

    def ingest_rids_through(timestamp):
        nonlocal rid_position
        while (
            rid_position < len(rid_records)
            and float(rid_records[rid_position]["receive_ts"]) <= timestamp
        ):
            rid_state.ingest(rid_records[rid_position])
            rid_position += 1
        registry.observe_rids(rid_state.snapshots(), now_ts=timestamp)

    hit_counts = {}

    def observation_from_event(timestamp, row, force_confirmed=False):
        track_id = int(row["track_id"])
        measurement_index = _int(row.get("meas_idx"), 0)
        source = measurements.nearest(measurement_index, timestamp) or {}
        relative_az = float(row["meas_az"])
        elevation = float(row["meas_el"])
        # The event CSV records detector measurements, not the track's Kalman
        # velocity state.  Treat direction as unavailable instead of deriving
        # a noisy one-frame velocity and pretending it is SORT state.
        velocity_az = 0.0
        velocity_el = 0.0
        hit_counts[track_id] = hit_counts.get(track_id, 0) + 1
        logged_hits = _int(row.get("hit_streak"), 0)
        hit_streak = max(hit_counts[track_id], logged_hits)
        return SortObservation(
            generation=SortGeneration(
                track_id,
                float(created_by_track.get(track_id, start_ts)),
            ),
            ui_confirmed=bool(force_confirmed or hit_streak >= 16),
            map_az=(relative_az + float(device_heading_deg)) % 360.0,
            elevation=elevation,
            velocity_az=velocity_az,
            velocity_el=velocity_el,
            board=str(source.get("board", "")),
            camera_id=_int(source.get("cam"), -1),
            logic_id=_int(source.get("logic_id"), -1),
            last_detection_ts=timestamp,
            hit_streak=hit_streak,
        )

    seed_ts, seed_row = seed
    ingest_rids_through(seed_ts)
    seed_snapshots = [
        item for item in rid_state.snapshots() if item["rid_id"] == XDB_RID
    ]
    if not seed_snapshots:
        raise ValueError("XDB RID data was not available at the replay checkpoint")
    registry.observe_rids(seed_snapshots, now_ts=seed_ts)
    registry.observe_sorts(
        [observation_from_event(seed_ts, seed_row, force_confirmed=True)],
        station,
        now_ts=seed_ts,
    )

    seed_consumed = False
    for timestamp, row in replay_events:
        if not seed_consumed and timestamp == seed_ts and row is seed_row:
            seed_consumed = True
            continue
        ingest_rids_through(timestamp)
        registry.observe_sorts(
            [observation_from_event(timestamp, row)],
            station,
            now_ts=timestamp,
        )

    ui_ids = {
        slot.rid_id: slot.ui_id for slot in registry.registered_slots()
    }
    sort_families = {
        slot.rid_id: {generation.sort_id for generation in slot.sort_family}
        for slot in registry.registered_slots()
    }
    owner_by_sort = {}
    for generation in registry.owned_sort_generations():
        owner = registry.owner_of(generation)
        if owner is not None:
            owner_by_sort[generation.sort_id] = owner.rid_id
    current_sort_by_rid = {
        slot.rid_id: (
            None if slot.current_sort is None else slot.current_sort.sort_id
        )
        for slot in registry.registered_slots()
    }
    return ReplayResult(
        registration_order=registration_order,
        ui_ids=ui_ids,
        sort_families=sort_families,
        owner_by_sort=owner_by_sort,
        current_sort_by_rid=current_sort_by_rid,
        forbidden_steal_count=registry.ownership_reject_count,
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", required=True, type=Path)
    parser.add_argument("--start-ts", type=float, default=1789156230.355754)
    parser.add_argument("--device-heading-deg", type=float, default=180.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    result = replay_run(
        args.log_dir,
        start_ts=args.start_ts,
        device_heading_deg=args.device_heading_deg,
    )
    payload = result.json_ready()
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
        return 0
    print(f"registration_order={result.registration_order}")
    print(f"ui_ids={result.ui_ids}")
    print(f"sort_families={result.sort_families}")
    print(f"current_sort_by_rid={result.current_sort_by_rid}")
    print(f"owner_by_sort={result.owner_by_sort}")
    print(f"forbidden_steal_count={result.forbidden_steal_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
