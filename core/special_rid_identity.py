"""Stable UI identities for the two configured RID_laser aircraft.

This module deliberately stays independent from SORT control and strike
selection.  SORT contributes only visual continuity and camera provenance;
all position fields exported for a special RID are derived from RID geometry.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import hmac
import itertools
import math
import threading
import time
from typing import Mapping


EARTH_RADIUS_M = 6_371_008.8
RID_LASER_SHA256_DIGESTS = frozenset({
    bytes.fromhex(
        "e22bf88acbc6464a26dfb6c955cb4e984d9bd2844e9bd518582ed46ba2dc19f1"
    ),
    bytes.fromhex(
        "1b52e697bd295126532a327129f9dcf407399eeaad35bbebf822693d6fdd4125"
    ),
})
SPECIAL_RID_UI_IDS = frozenset((1, 2))
REPLACEABLE_VISUAL_UI_IDS = frozenset((3, 4, 5))
REPLACEMENT_SUCCESS_COUNT = 3


def is_rid_laser(rid_id, digests=None):
    """Return whether *rid_id* belongs to the configured RID_laser set."""
    candidate = hashlib.sha256(str(rid_id).encode("utf-8")).digest()
    allowed = RID_LASER_SHA256_DIGESTS if digests is None else digests
    return any(hmac.compare_digest(candidate, digest) for digest in allowed)


def _finite_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _project_geodetic(latitude, longitude, reference_latitude):
    latitude_rad = math.radians(float(latitude))
    longitude_rad = math.radians(float(longitude))
    scale = math.cos(math.radians(float(reference_latitude)))
    return (
        longitude_rad * EARTH_RADIUS_M * scale,
        latitude_rad * EARTH_RADIUS_M,
    )


def _unproject_geodetic(east_m, north_m, reference_latitude):
    scale = math.cos(math.radians(float(reference_latitude)))
    latitude = math.degrees(float(north_m) / EARTH_RADIUS_M)
    longitude = math.degrees(float(east_m) / (EARTH_RADIUS_M * scale))
    return latitude, longitude


def _reported_horizontal_velocity(sample, fallback, max_speed_mps):
    speed = _finite_float(sample.get("horizontal_speed"))
    heading = _finite_float(sample.get("track_heading"))
    if speed is None:
        speed = _finite_float(fallback.get("horizontal_speed"))
    if heading is None:
        heading = _finite_float(fallback.get("track_heading"))
    if (
        speed is None
        or heading is None
        or speed < 0.0
        or speed > float(max_speed_mps)
        or not 0.0 <= heading < 360.0
    ):
        return None
    heading_rad = math.radians(heading)
    return speed * math.sin(heading_rad), speed * math.cos(heading_rad)


def _reported_relative_height(sample, rid_item):
    """Read RID Height only for the matching latest measurement.

    The compiled RID manager exposes Height on the top-level snapshot but its
    measurement_history entries do not contain that field.  Do not apply the
    latest Height retroactively to older history entries.
    """
    sample_height = _finite_float(sample.get("height"))
    if sample_height is not None:
        return sample_height
    try:
        sample_seq = int(sample.get("measurement_seq", 0))
        latest_seq = int(rid_item.get("measurement_seq", 0))
    except (TypeError, ValueError, OverflowError):
        return None
    if sample_seq != latest_seq:
        return None
    return _finite_float(rid_item.get("height"))


@dataclass(frozen=True)
class PredictedRidPoint:
    latitude: float
    longitude: float
    relative_height_m: float | None
    east_m: float
    north_m: float
    mode: str
    prediction_age_s: float
    receive_age_s: float
    measurement_age_s: float


@dataclass(frozen=True, order=True)
class SortGeneration:
    sort_id: int
    created_ts: float


@dataclass(frozen=True)
class SortObservation:
    generation: SortGeneration
    ui_confirmed: bool
    map_az: float
    elevation: float
    velocity_az: float
    velocity_el: float
    board: str
    camera_id: int
    logic_id: int
    last_detection_ts: float
    hit_streak: int
    replaced_visual_ui_id: int = 0


@dataclass(frozen=True)
class SpecialRidUIStatus:
    board: str
    camera_id: int
    target_id: int
    azimuth: float
    elevation: float
    distance: float
    threat_score: float
    replaced_target_id: int = 0


@dataclass
class SpecialRidSlot:
    rid_id: str
    rid_key: object
    predictor: "SpecialRidPredictor"
    pending_since: float
    ui_id: int | None = None
    registration_state: str = "pending"
    current_sort: SortGeneration | None = None
    sort_family: set | None = None
    last_visual_detection_ts: float = 0.0
    visual_missing_since: float | None = None
    replacement_queue: list = field(default_factory=list)
    replacement_generations: set = field(default_factory=set)

    def __post_init__(self):
        if self.sort_family is None:
            self.sort_family = set()


class SpecialRidPredictor:
    """Causal alpha-beta RID predictor with no display delay."""

    def __init__(
        self,
        *,
        alpha=0.85,
        beta=0.18,
        max_speed_mps=40.0,
        max_prediction_s=5.0,
        fresh_s=7.0,
    ):
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.beta = max(0.0, float(beta))
        self.max_speed_mps = max(0.1, float(max_speed_mps))
        self.max_prediction_s = max(0.0, float(max_prediction_s))
        self.fresh_s = max(self.max_prediction_s, float(fresh_s))
        self.rid_key = None
        self.reference_latitude = None
        self.east_m = 0.0
        self.north_m = 0.0
        self.relative_height_m = None
        self.velocity_east_mps = 0.0
        self.velocity_north_mps = 0.0
        self.velocity_vertical_mps = 0.0
        self.state_ts = None
        self.last_measurement_ts = None
        self.last_measurement_seq = 0
        self.last_receive_ts = None
        self.measurement_count = 0
        self.measured_east_m = 0.0
        self.measured_north_m = 0.0
        self.last_update_mode = "uninitialized"

    def _reset_from_sample(self, sample, rid_item, measurement_seq, timestamp):
        latitude = _finite_float(sample.get("latitude"))
        longitude = _finite_float(sample.get("longitude"))
        if latitude is None or longitude is None:
            return False
        self.reference_latitude = latitude
        east_m, north_m = _project_geodetic(latitude, longitude, latitude)
        reported = _reported_horizontal_velocity(
            sample, rid_item, self.max_speed_mps
        )
        self.east_m = east_m
        self.north_m = north_m
        self.measured_east_m = east_m
        self.measured_north_m = north_m
        self.velocity_east_mps, self.velocity_north_mps = reported or (0.0, 0.0)
        self.relative_height_m = _reported_relative_height(sample, rid_item)
        vertical_speed = _finite_float(sample.get("vertical_speed"))
        if vertical_speed is None:
            vertical_speed = _finite_float(rid_item.get("vertical_speed"))
        self.velocity_vertical_mps = vertical_speed or 0.0
        self.state_ts = timestamp
        self.last_measurement_ts = timestamp
        self.last_measurement_seq = measurement_seq
        self.measurement_count += 1
        self.last_update_mode = "reset"
        return True

    def _observe_sample(self, sample, rid_item):
        try:
            measurement_seq = int(sample.get("measurement_seq", 0))
        except (TypeError, ValueError, OverflowError):
            return False
        timestamp = _finite_float(sample.get("timestamp"))
        latitude = _finite_float(sample.get("latitude"))
        longitude = _finite_float(sample.get("longitude"))
        if (
            measurement_seq <= self.last_measurement_seq
            or timestamp is None
            or latitude is None
            or longitude is None
        ):
            return False
        if self.state_ts is None:
            return self._reset_from_sample(
                sample, rid_item, measurement_seq, timestamp
            )

        east_m, north_m = _project_geodetic(
            latitude, longitude, self.reference_latitude
        )
        gap_s = timestamp - float(self.last_measurement_ts)
        if gap_s <= 1.0e-6 or gap_s > self.fresh_s:
            return self._reset_from_sample(
                sample, rid_item, measurement_seq, timestamp
            )

        predicted_east = self.east_m + self.velocity_east_mps * gap_s
        predicted_north = self.north_m + self.velocity_north_mps * gap_s
        residual_east = east_m - predicted_east
        residual_north = north_m - predicted_north
        measured_speed = math.hypot(
            east_m - self.measured_east_m,
            north_m - self.measured_north_m,
        ) / gap_s
        if measured_speed > self.max_speed_mps * 2.0:
            return self._reset_from_sample(
                sample, rid_item, measurement_seq, timestamp
            )

        self.east_m = predicted_east + self.alpha * residual_east
        self.north_m = predicted_north + self.alpha * residual_north
        self.velocity_east_mps += self.beta * residual_east / gap_s
        self.velocity_north_mps += self.beta * residual_north / gap_s
        reported = _reported_horizontal_velocity(
            sample, rid_item, self.max_speed_mps
        )
        if reported is not None:
            self.velocity_east_mps = (
                0.75 * self.velocity_east_mps + 0.25 * reported[0]
            )
            self.velocity_north_mps = (
                0.75 * self.velocity_north_mps + 0.25 * reported[1]
            )
        speed = math.hypot(self.velocity_east_mps, self.velocity_north_mps)
        if speed > self.max_speed_mps:
            scale = self.max_speed_mps / speed
            self.velocity_east_mps *= scale
            self.velocity_north_mps *= scale

        relative_height_m = _reported_relative_height(sample, rid_item)
        if relative_height_m is not None:
            if self.relative_height_m is None:
                self.relative_height_m = relative_height_m
            else:
                predicted_height = (
                    self.relative_height_m + self.velocity_vertical_mps * gap_s
                )
                residual_height = relative_height_m - predicted_height
                self.relative_height_m = (
                    predicted_height + self.alpha * residual_height
                )
                self.velocity_vertical_mps += self.beta * residual_height / gap_s
        reported_vertical = _finite_float(sample.get("vertical_speed"))
        if reported_vertical is None:
            reported_vertical = _finite_float(rid_item.get("vertical_speed"))
        if reported_vertical is not None:
            self.velocity_vertical_mps = (
                0.75 * self.velocity_vertical_mps + 0.25 * reported_vertical
            )

        self.measured_east_m = east_m
        self.measured_north_m = north_m
        self.state_ts = timestamp
        self.last_measurement_ts = timestamp
        self.last_measurement_seq = measurement_seq
        self.measurement_count += 1
        self.last_update_mode = "measurement"
        return True

    def observe(self, rid_item: Mapping) -> bool:
        if not isinstance(rid_item, Mapping):
            return False
        receive_ts = _finite_float(rid_item.get("last_receive_ts"))
        if receive_ts is not None:
            self.last_receive_ts = receive_ts
        if not bool(rid_item.get("position_valid", False)):
            return False

        rid_key = rid_item.get("key")
        if self.rid_key is None:
            self.rid_key = rid_key
        elif rid_key != self.rid_key:
            return False

        samples = sorted(
            rid_item.get("measurement_history") or (),
            key=lambda item: int(item.get("measurement_seq", 0)),
        )
        changed = False
        for sample in samples:
            changed = self._observe_sample(sample, rid_item) or changed
        return changed

    def predict(self, now_ts: float) -> PredictedRidPoint | None:
        now_ts = float(now_ts)
        if self.state_ts is None or self.last_receive_ts is None:
            return None
        receive_age_s = max(0.0, now_ts - self.last_receive_ts)
        if receive_age_s > self.fresh_s:
            return None
        measurement_age_s = max(0.0, now_ts - self.last_measurement_ts)
        prediction_age_s = min(measurement_age_s, self.max_prediction_s)
        east_m = self.east_m + self.velocity_east_mps * prediction_age_s
        north_m = self.north_m + self.velocity_north_mps * prediction_age_s
        relative_height_m = (
            None
            if self.relative_height_m is None
            else self.relative_height_m
            + self.velocity_vertical_mps * prediction_age_s
        )
        latitude, longitude = _unproject_geodetic(
            east_m, north_m, self.reference_latitude
        )
        if measurement_age_s <= 1.0e-6:
            mode = "measurement"
        elif measurement_age_s <= self.max_prediction_s:
            mode = "prediction"
        else:
            mode = "freeze"
        return PredictedRidPoint(
            latitude=latitude,
            longitude=longitude,
            relative_height_m=relative_height_m,
            east_m=east_m,
            north_m=north_m,
            mode=mode,
            prediction_age_s=prediction_age_s,
            receive_age_s=receive_age_s,
            measurement_age_s=measurement_age_s,
        )


def _wrapped_delta_deg(left, right):
    return (float(left) - float(right) + 180.0) % 360.0 - 180.0


def _angle_distance(az_a, el_a, az_b, el_b):
    return math.hypot(
        _wrapped_delta_deg(az_a, az_b),
        float(el_a) - float(el_b),
    )


def _geometry_from_point(point, station):
    if not isinstance(station, Mapping) or not bool(station.get("valid", False)):
        return None
    station_lat = _finite_float(station.get("latitude"))
    station_lon = _finite_float(station.get("longitude"))
    if (
        station_lat is None
        or station_lon is None
        or point.relative_height_m is None
    ):
        return None
    lat1 = math.radians(station_lat)
    lat2 = math.radians(point.latitude)
    delta_lat = lat2 - lat1
    delta_lon = math.radians(point.longitude - station_lon)
    hav = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2
    )
    horizontal_m = 2.0 * EARTH_RADIUS_M * math.asin(
        min(1.0, math.sqrt(max(0.0, hav)))
    )
    bearing_y = math.sin(delta_lon) * math.cos(lat2)
    bearing_x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    )
    azimuth = math.degrees(math.atan2(bearing_y, bearing_x)) % 360.0
    # For the two special RID aircraft, the broadcast Height is already the
    # vertical difference used by UI and strike geometry.  Station altitude
    # and RID AltGeo are deliberately excluded from this path.
    vertical_m = float(point.relative_height_m)
    elevation = math.degrees(math.atan2(vertical_m, horizontal_m))
    distance = math.hypot(horizontal_m, vertical_m)
    if not all(math.isfinite(value) for value in (azimuth, elevation, distance)):
        return None
    return azimuth, elevation, distance


def sort_generation_from_track(track):
    return SortGeneration(int(track.id), float(track.created_ts))


def should_send_sort_through_ordinary_ui(track, owned_generations):
    """Keep special-owned SORT generations out of the legacy UI path."""

    return sort_generation_from_track(track) not in set(owned_generations or ())


def sort_observation_from_track(
    track,
    map_azimuth,
    replaced_visual_ui_id=0,
):
    """Copy only stable SORT state needed by the special identity layer."""

    board = getattr(track, "last_source_board", None)
    camera_id = getattr(track, "last_source_cam", None)
    logic_id = getattr(track, "last_source_logic_id", None)
    try:
        camera_id = int(camera_id)
    except (TypeError, ValueError, OverflowError):
        camera_id = -1
    try:
        logic_id = int(logic_id)
    except (TypeError, ValueError, OverflowError):
        logic_id = -1
    relative_az = float(track.state[0, 0])
    return SortObservation(
        generation=sort_generation_from_track(track),
        ui_confirmed=bool(getattr(track, "ui_confirmed", False)),
        map_az=float(map_azimuth(relative_az)),
        elevation=float(track.state[1, 0]),
        velocity_az=float(track.state[2, 0]),
        velocity_el=float(track.state[3, 0]),
        board="" if board is None else str(board),
        camera_id=camera_id,
        logic_id=logic_id,
        last_detection_ts=float(getattr(track, "last_update_ts", 0.0)),
        hit_streak=int(getattr(track, "hit_streak", 0)),
        replaced_visual_ui_id=int(replaced_visual_ui_id or 0),
    )


class SpecialRidRegistry:
    """Own process-lifetime UI slots and SORT generations for RID_laser."""

    def __init__(
        self,
        *,
        predictor_factory=SpecialRidPredictor,
        log_callback=None,
        camera_theta=None,
        sort_fresh_s=6.0,
        sort_internal_s=12.0,
        reacquire_delay_s=0.0,
        rid_laser_digests=None,
    ):
        self.predictor_factory = predictor_factory
        self.log_callback = log_callback
        self.camera_theta = dict(camera_theta or {})
        self.sort_fresh_s = max(0.1, float(sort_fresh_s))
        self.sort_internal_s = max(self.sort_fresh_s, float(sort_internal_s))
        self.reacquire_delay_s = max(0.0, float(reacquire_delay_s))
        self.rid_laser_digests = (
            RID_LASER_SHA256_DIGESTS
            if rid_laser_digests is None
            else frozenset(bytes(digest) for digest in rid_laser_digests)
        )
        self._lock = threading.RLock()
        self._slots = {}
        self._owners = {}
        self._sort_observations = {}
        self._next_ui_id = 1
        self._ownership_rejections = set()
        self.ownership_reject_count = 0

    def _log(self, event, **values):
        if self.log_callback is None:
            return
        row = {"event": event, **values}
        try:
            self.log_callback(row)
        except Exception:
            pass

    def observe_rids(self, rid_items, now_ts):
        now_ts = float(now_ts)
        with self._lock:
            for item in rid_items or ():
                rid_id = str(item.get("rid_id", ""))
                if not is_rid_laser(rid_id, self.rid_laser_digests):
                    continue
                slot = self._slots.get(rid_id)
                if slot is None:
                    if not bool(item.get("position_valid", False)):
                        continue
                    predictor = self.predictor_factory()
                    pending_since = _finite_float(item.get("last_receive_ts"))
                    if pending_since is None:
                        pending_since = now_ts
                    slot = SpecialRidSlot(
                        rid_id=rid_id,
                        rid_key=item.get("key"),
                        predictor=predictor,
                        pending_since=pending_since,
                    )
                    self._slots[rid_id] = slot
                    self._log(
                        "REGISTER_PENDING",
                        timestamp=now_ts,
                        rid_id=rid_id,
                        registration_state="pending",
                    )
                slot.predictor.observe(item)

    def _registration_assignments(self, pending, candidates, station, now_ts):
        geometries = {}
        for slot in pending:
            point = slot.predictor.predict(now_ts)
            geometry = None if point is None else _geometry_from_point(point, station)
            if geometry is not None:
                geometries[slot.rid_id] = geometry
        pending = [slot for slot in pending if slot.rid_id in geometries]
        if not pending or not candidates:
            return []

        # Under the staged-launch workflow, one fresh special RID and one
        # confirmed unowned SORT are unambiguous even when SORT was created
        # just before the RID registry observed its first packet.
        force_singleton = len(pending) == 1 and len(candidates) == 1

        maximum = min(len(pending), len(candidates))
        for assignment_size in range(maximum, 0, -1):
            best = None
            for slot_subset in itertools.combinations(pending, assignment_size):
                for candidate_subset in itertools.permutations(
                    candidates, assignment_size
                ):
                    pairs = []
                    total = 0.0
                    valid = True
                    for slot, observation in zip(slot_subset, candidate_subset):
                        if (
                            observation.generation.created_ts < slot.pending_since
                            and not force_singleton
                        ):
                            valid = False
                            break
                        rid_az, rid_el, _ = geometries[slot.rid_id]
                        cost = _angle_distance(
                            observation.map_az,
                            observation.elevation,
                            rid_az,
                            rid_el,
                        )
                        total += cost
                        pairs.append((slot, observation, cost))
                    if not valid:
                        continue
                    tie = tuple(
                        (
                            pair[1].generation.created_ts,
                            pair[0].pending_since,
                            pair[0].rid_id,
                        )
                        for pair in pairs
                    )
                    rank = (total, tie)
                    if best is None or rank < best[0]:
                        best = (rank, pairs)
            if best is not None:
                return best[1]
        return []

    def _attach(
        self,
        slot,
        observation,
        now_ts,
        reason,
        angle_cost=math.nan,
        forced_binding=False,
    ):
        generation = observation.generation
        owner = self._owners.get(generation)
        if owner is not None and owner != slot.rid_id:
            self._log(
                "SORT_OWNERSHIP_REJECT",
                timestamp=now_ts,
                rid_id=slot.rid_id,
                candidate_sort_id=generation.sort_id,
                skip_reason=f"owned_by={owner}",
            )
            return False
        self._owners[generation] = slot.rid_id
        slot.sort_family.add(generation)
        self._sort_observations[generation] = observation
        if slot.current_sort is None:
            slot.current_sort = generation
        slot.last_visual_detection_ts = max(
            slot.last_visual_detection_ts, observation.last_detection_ts
        )
        if slot.ui_id is None:
            slot.ui_id = self._next_ui_id
            self._next_ui_id += 1
            slot.registration_state = "registered"
            self._log(
                "REGISTERED",
                timestamp=now_ts,
                rid_id=slot.rid_id,
                ui_id=slot.ui_id,
                registration_state=slot.registration_state,
                current_sort_id=generation.sort_id,
                current_sort_created_ts=generation.created_ts,
                angle_cost=angle_cost,
                forced_binding=1 if forced_binding else 0,
            )
        else:
            self._log(
                "SORT_FAMILY_ATTACH",
                timestamp=now_ts,
                rid_id=slot.rid_id,
                ui_id=slot.ui_id,
                current_sort_id=(
                    "" if slot.current_sort is None else slot.current_sort.sort_id
                ),
                candidate_sort_id=generation.sort_id,
                angle_cost=angle_cost,
                skip_reason=reason,
            )
        self._queue_visual_replacement(slot, observation, now_ts)
        return True

    def _queue_visual_replacement(self, slot, observation, now_ts):
        try:
            old_ui_id = int(observation.replaced_visual_ui_id)
        except (TypeError, ValueError, OverflowError):
            return
        generation = observation.generation
        if (
            old_ui_id not in REPLACEABLE_VISUAL_UI_IDS
            or old_ui_id == slot.ui_id
            or generation in slot.replacement_generations
        ):
            return
        slot.replacement_generations.add(generation)
        slot.replacement_queue.append(
            {
                "generation": generation,
                "old_ui_id": old_ui_id,
                "remaining": REPLACEMENT_SUCCESS_COUNT,
            }
        )
        self._log(
            "VISUAL_UI_REPLACEMENT_QUEUED",
            timestamp=now_ts,
            rid_id=slot.rid_id,
            ui_id=slot.ui_id,
            current_sort_id=generation.sort_id,
            candidate_sort_id=old_ui_id,
            skip_reason=f"successes_required={REPLACEMENT_SUCCESS_COUNT}",
        )

    def _camera_relation(self, left, right):
        if int(left.logic_id) == int(right.logic_id):
            return "same"
        left_index = int(left.logic_id) - 1
        right_index = int(right.logic_id) - 1
        if left_index < 0 or right_index < 0:
            return "different"
        left_layer, left_column = divmod(left_index, 5)
        right_layer, right_column = divmod(right_index, 5)
        if abs(left_layer - right_layer) > 1 or abs(left_column - right_column) > 1:
            return "different"
        left_theta = self.camera_theta.get(int(left.logic_id))
        right_theta = self.camera_theta.get(int(right.logic_id))
        if left_theta is not None and right_theta is not None:
            horizontal = abs(
                _wrapped_delta_deg(
                    left_theta.get("theta_horizontal", 0.0),
                    right_theta.get("theta_horizontal", 0.0),
                )
            )
            vertical = abs(
                float(left_theta.get("theta_vertical", 0.0))
                - float(right_theta.get("theta_vertical", 0.0))
            )
            if horizontal > 30.0 or vertical > 15.0:
                return "different"
        return "adjacent"

    @staticmethod
    def _direction_difference(left, right):
        left_speed = math.hypot(left.velocity_az, left.velocity_el)
        right_speed = math.hypot(right.velocity_az, right.velocity_el)
        if left_speed < 0.1 or right_speed < 0.1:
            return 0.0
        left_direction = math.degrees(
            math.atan2(left.velocity_el, left.velocity_az)
        )
        right_direction = math.degrees(
            math.atan2(right.velocity_el, right.velocity_az)
        )
        return abs(_wrapped_delta_deg(left_direction, right_direction))

    def _match_to_slot(self, slot, candidate):
        if candidate.generation.created_ts < slot.pending_since:
            return None
        best = None
        for generation in slot.sort_family:
            previous = self._sort_observations.get(generation)
            if previous is None:
                continue
            if (
                candidate.generation.created_ts
                <= previous.generation.created_ts
            ):
                continue
            direct_residual = _angle_distance(
                previous.map_az,
                previous.elevation,
                candidate.map_az,
                candidate.elevation,
            )
            simultaneous_gap = abs(
                candidate.last_detection_ts - previous.last_detection_ts
            )
            relation = self._camera_relation(previous, candidate)
            time_gap = max(
                0.0,
                candidate.generation.created_ts - previous.last_detection_ts,
            )
            if simultaneous_gap <= self.sort_internal_s and direct_residual <= 1.5:
                match_class = 0
                residual = direct_residual
                relation_name = "converged"
            else:
                predicted_az = previous.map_az
                predicted_el = previous.elevation
                if 0.0 <= time_gap <= self.sort_internal_s:
                    predicted_az = (
                        predicted_az + previous.velocity_az * time_gap
                    ) % 360.0
                    predicted_el += previous.velocity_el * time_gap
                residual = _angle_distance(
                    predicted_az,
                    predicted_el,
                    candidate.map_az,
                    candidate.elevation,
                )
                if relation == "same" and residual <= 6.0:
                    match_class = 1
                    relation_name = "same_camera"
                elif relation == "adjacent" and residual <= 9.0:
                    match_class = 2
                    relation_name = "adjacent_camera"
                else:
                    continue
            direction_difference = self._direction_difference(previous, candidate)
            rank = (
                match_class,
                residual,
                direction_difference,
                time_gap,
                -candidate.last_detection_ts,
                -candidate.hit_streak,
                candidate.generation.sort_id,
                candidate.generation.created_ts,
            )
            match = {
                "rank": rank,
                "residual": residual,
                "direction_difference": direction_difference,
                "time_gap": time_gap,
                "camera_relation": relation_name,
                "previous": previous,
            }
            if best is None or rank < best["rank"]:
                best = match
        return best

    def _attach_normal_successors(self, observations, now_ts):
        candidates = sorted(
            (
                observation
                for observation in observations
                if observation.generation not in self._owners
            ),
            key=lambda item: (
                item.generation.created_ts,
                item.generation.sort_id,
            ),
        )
        for candidate in candidates:
            choices = []
            for slot in self._slots.values():
                if slot.ui_id is None:
                    continue
                match = self._match_to_slot(slot, candidate)
                if match is not None:
                    current_observation = self._sort_observations.get(
                        slot.current_sort
                    )
                    current_is_fresh = (
                        current_observation is not None
                        and current_observation.ui_confirmed
                        and (now_ts - current_observation.last_detection_ts)
                        <= self.sort_fresh_s
                    )
                    if not current_is_fresh:
                        self._log(
                            "SUCCESSOR_CANDIDATE",
                            timestamp=now_ts,
                            rid_id=slot.rid_id,
                            ui_id=slot.ui_id,
                            current_sort_id=(
                                "" if slot.current_sort is None else slot.current_sort.sort_id
                            ),
                            candidate_sort_id=candidate.generation.sort_id,
                            angle_cost=match["residual"],
                            camera_relation=match["camera_relation"],
                            time_gap_s=match["time_gap"],
                            forced_binding=0,
                            skip_reason="current_sort_stale_use_rid_reacquire",
                        )
                        continue
                    if match["rank"][0] != 0:
                        self._log(
                            "SUCCESSOR_CANDIDATE",
                            timestamp=now_ts,
                            rid_id=slot.rid_id,
                            ui_id=slot.ui_id,
                            current_sort_id=slot.current_sort.sort_id,
                            candidate_sort_id=candidate.generation.sort_id,
                            angle_cost=match["residual"],
                            camera_relation=match["camera_relation"],
                            time_gap_s=match["time_gap"],
                            forced_binding=0,
                            skip_reason="current_sort_still_fresh",
                        )
                        continue
                    choices.append((match["rank"], slot, match))
                    self._log(
                        "SUCCESSOR_CANDIDATE",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        current_sort_id=(
                            "" if slot.current_sort is None else slot.current_sort.sort_id
                        ),
                        candidate_sort_id=candidate.generation.sort_id,
                        angle_cost=match["residual"],
                        camera_relation=match["camera_relation"],
                        time_gap_s=match["time_gap"],
                        forced_binding=0,
                    )
            if not choices:
                continue
            _, slot, match = min(choices, key=lambda item: item[0])
            self._attach(
                slot,
                candidate,
                now_ts,
                reason=match["camera_relation"],
                angle_cost=match["residual"],
            )

    def _reacquire_missing_slots(self, observations, station, now_ts):
        waiting_slots = []
        for slot in self._slots.values():
            if slot.ui_id is None:
                continue
            current = self._sort_observations.get(slot.current_sort)
            current_is_fresh = (
                current is not None
                and current.ui_confirmed
                and (now_ts - current.last_detection_ts) <= self.sort_fresh_s
            )
            stale_for_s = max(
                0.0,
                now_ts - slot.last_visual_detection_ts - self.sort_fresh_s,
            )
            if current_is_fresh or stale_for_s < self.reacquire_delay_s:
                continue
            waiting_slots.append(slot)

        for slot in waiting_slots:
            for observation in observations:
                owner_rid = self._owners.get(observation.generation)
                rejection_key = (slot.rid_id, observation.generation)
                if (
                    owner_rid is None
                    or owner_rid == slot.rid_id
                    or rejection_key in self._ownership_rejections
                ):
                    continue
                self._ownership_rejections.add(rejection_key)
                self.ownership_reject_count += 1
                self._log(
                    "SORT_OWNERSHIP_REJECT",
                    timestamp=now_ts,
                    rid_id=slot.rid_id,
                    ui_id=slot.ui_id,
                    candidate_sort_id=observation.generation.sort_id,
                    skip_reason=f"owned_by={owner_rid}",
                    forced_binding=1,
                )
        candidates = [
            observation
            for observation in observations
            if observation.generation not in self._owners
        ]
        assignments = self._registration_assignments(
            waiting_slots, candidates, station, now_ts
        )
        assignments.sort(
            key=lambda pair: (
                pair[1].generation.created_ts,
                pair[0].pending_since,
                pair[0].rid_id,
            )
        )
        for slot, candidate, angle_cost in assignments:
            if self._attach(
                slot,
                candidate,
                now_ts,
                reason="immediate_nearest_rid",
                angle_cost=angle_cost,
            ):
                self._log(
                    "IMMEDIATE_REACQUIRE",
                    timestamp=now_ts,
                    rid_id=slot.rid_id,
                    ui_id=slot.ui_id,
                    candidate_sort_id=candidate.generation.sort_id,
                    angle_cost=angle_cost,
                    forced_binding=1,
                )

    def select_current_sorts(self, now_ts):
        now_ts = float(now_ts)
        with self._lock:
            for slot in self._slots.values():
                if slot.ui_id is None:
                    continue
                current_observation = self._sort_observations.get(slot.current_sort)
                if (
                    current_observation is not None
                    and current_observation.ui_confirmed
                    and (now_ts - current_observation.last_detection_ts)
                    <= self.sort_fresh_s
                ):
                    continue
                fresh = [
                    self._sort_observations[generation]
                    for generation in slot.sort_family
                    if generation in self._sort_observations
                    and self._sort_observations[generation].ui_confirmed
                    and (now_ts - self._sort_observations[generation].last_detection_ts)
                    <= self.sort_fresh_s
                ]
                old_generation = slot.current_sort
                if fresh:
                    selected = min(
                        fresh,
                        key=lambda item: (
                            -item.last_detection_ts,
                            -item.hit_streak,
                            item.generation.sort_id,
                            item.generation.created_ts,
                        ),
                    )
                    slot.current_sort = selected.generation
                    slot.last_visual_detection_ts = max(
                        slot.last_visual_detection_ts,
                        selected.last_detection_ts,
                    )
                    slot.visual_missing_since = None
                    if old_generation != selected.generation:
                        self._log(
                            "CURRENT_SORT_SWITCH",
                            timestamp=now_ts,
                            rid_id=slot.rid_id,
                            ui_id=slot.ui_id,
                            current_sort_id=selected.generation.sort_id,
                            current_sort_created_ts=selected.generation.created_ts,
                            candidate_sort_id=(
                                "" if old_generation is None else old_generation.sort_id
                            ),
                        )
                elif slot.visual_missing_since is None:
                    slot.visual_missing_since = now_ts
                    self._log(
                        "VISUAL_LOST",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        current_sort_id=(
                            "" if old_generation is None else old_generation.sort_id
                        ),
                    )

    def observe_sorts(self, sort_items, station, now_ts):
        now_ts = float(now_ts)
        with self._lock:
            observations = []
            for observation in sort_items or ():
                if not isinstance(observation, SortObservation):
                    continue
                self._sort_observations[observation.generation] = observation
                if observation.ui_confirmed:
                    observations.append(observation)

            fresh_observations = [
                observation
                for observation in observations
                if 0.0 <= (now_ts - observation.last_detection_ts)
                <= self.sort_fresh_s
            ]

            pending = [slot for slot in self._slots.values() if slot.ui_id is None]
            unowned = [
                observation
                for observation in fresh_observations
                if observation.generation not in self._owners
            ]
            assignments = self._registration_assignments(
                pending, unowned, station, now_ts
            )
            force_singleton = len(pending) == 1 and len(unowned) == 1
            assignments.sort(
                key=lambda pair: (
                    pair[1].generation.created_ts,
                    pair[0].pending_since,
                    pair[0].rid_id,
                )
            )
            for slot, observation, angle_cost in assignments:
                self._attach(
                    slot,
                    observation,
                    now_ts,
                    reason=(
                        "initial_single_rid_single_sort"
                        if force_singleton
                        else "initial_nearest_2d_angle"
                    ),
                    angle_cost=angle_cost,
                    forced_binding=force_singleton,
                )
            for observation in observations:
                owner_rid = self._owners.get(observation.generation)
                if owner_rid is None:
                    continue
                slot = self._slots.get(owner_rid)
                if slot is None:
                    continue
                slot.last_visual_detection_ts = max(
                    slot.last_visual_detection_ts,
                    observation.last_detection_ts,
                )
            self.select_current_sorts(now_ts)
            self._attach_normal_successors(fresh_observations, now_ts)
            self._reacquire_missing_slots(
                fresh_observations, station, now_ts
            )
            self.select_current_sorts(now_ts)

    def slot_for_rid(self, rid_id):
        with self._lock:
            return self._slots.get(str(rid_id))

    def registered_slots(self):
        with self._lock:
            return tuple(
                sorted(
                    (slot for slot in self._slots.values() if slot.ui_id is not None),
                    key=lambda slot: slot.ui_id,
                )
            )

    def owned_sort_generations(self):
        with self._lock:
            return frozenset(self._owners)

    def owner_of(self, generation):
        with self._lock:
            rid_id = self._owners.get(generation)
            return None if rid_id is None else self._slots.get(rid_id)

    def owner_of_sort_id(self, sort_id):
        sort_id = int(sort_id)
        with self._lock:
            generations = [
                generation
                for generation in self._owners
                if generation.sort_id == sort_id
            ]
            if not generations:
                return None
            generation = max(generations, key=lambda item: item.created_ts)
            return self._slots.get(self._owners[generation])

    def replacement_pending_for_generation(self, generation):
        with self._lock:
            rid_id = self._owners.get(generation)
            slot = None if rid_id is None else self._slots.get(rid_id)
            if slot is None:
                return False
            return any(
                task["generation"] == generation
                for task in slot.replacement_queue
            )

    def ack_ui_status_sent(self, target_id, replaced_target_id):
        try:
            target_id = int(target_id)
            replaced_target_id = int(replaced_target_id)
        except (TypeError, ValueError, OverflowError):
            return False
        if replaced_target_id not in REPLACEABLE_VISUAL_UI_IDS:
            return False
        with self._lock:
            slot = next(
                (
                    item
                    for item in self._slots.values()
                    if item.ui_id == target_id
                ),
                None,
            )
            if slot is None or not slot.replacement_queue:
                return False
            task = slot.replacement_queue[0]
            if task["old_ui_id"] != replaced_target_id:
                return False
            task["remaining"] -= 1
            if task["remaining"] <= 0:
                completed = slot.replacement_queue.pop(0)
                self._log(
                    "VISUAL_UI_REPLACEMENT_ACKED",
                    timestamp=time.time(),
                    rid_id=slot.rid_id,
                    ui_id=slot.ui_id,
                    current_sort_id=completed["generation"].sort_id,
                    candidate_sort_id=completed["old_ui_id"],
                )
            return True

    @staticmethod
    def _threat_score(distance):
        if distance < 100.0:
            return 100.0
        if distance <= 300.0:
            return 50.0
        return 0.0

    def ui_statuses(self, now_ts, station):
        now_ts = float(now_ts)
        with self._lock:
            self.select_current_sorts(now_ts)
            statuses = []
            for slot in sorted(
                self._slots.values(),
                key=lambda item: (
                    math.inf if item.ui_id is None else item.ui_id,
                    item.rid_id,
                ),
            ):
                if slot.ui_id is None:
                    continue
                observation = self._sort_observations.get(slot.current_sort)
                if observation is None or not observation.ui_confirmed:
                    self._log(
                        "UI_SKIP",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        skip_reason="no_confirmed_current_sort",
                    )
                    continue
                sort_age_s = max(0.0, now_ts - observation.last_detection_ts)
                if sort_age_s > self.sort_fresh_s:
                    self._log(
                        "UI_SKIP",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        current_sort_id=observation.generation.sort_id,
                        sort_age_s=sort_age_s,
                        skip_reason="sort_stale",
                    )
                    continue
                board = str(observation.board or "").strip()
                try:
                    camera_id = int(observation.camera_id)
                except (TypeError, ValueError, OverflowError):
                    camera_id = -1
                if not board or not 0 <= camera_id <= 255:
                    self._log(
                        "UI_SKIP",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        current_sort_id=observation.generation.sort_id,
                        skip_reason="invalid_sort_provenance",
                    )
                    continue
                point = slot.predictor.predict(now_ts)
                if point is None:
                    self._log(
                        "UI_SKIP",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        current_sort_id=observation.generation.sort_id,
                        sort_age_s=sort_age_s,
                        skip_reason="rid_stale_or_missing",
                    )
                    continue
                geometry = _geometry_from_point(point, station)
                if geometry is None or geometry[2] <= 0.0:
                    self._log(
                        "UI_SKIP",
                        timestamp=now_ts,
                        rid_id=slot.rid_id,
                        ui_id=slot.ui_id,
                        current_sort_id=observation.generation.sort_id,
                        sort_age_s=sort_age_s,
                        rid_receive_age_s=point.receive_age_s,
                        skip_reason="invalid_station_or_rid_geometry",
                    )
                    continue
                azimuth, elevation, distance = geometry
                threat_score = self._threat_score(distance)
                replaced_target_id = (
                    int(slot.replacement_queue[0]["old_ui_id"])
                    if slot.replacement_queue
                    else 0
                )
                self._log(
                    "RID_PREDICT",
                    timestamp=now_ts,
                    rid_id=slot.rid_id,
                    ui_id=slot.ui_id,
                    current_sort_id=observation.generation.sort_id,
                    current_sort_created_ts=observation.generation.created_ts,
                    board=board,
                    cam=camera_id,
                    logic_id=observation.logic_id,
                    sort_age_s=sort_age_s,
                    rid_receive_age_s=point.receive_age_s,
                    rid_measurement_age_s=point.measurement_age_s,
                    rid_point_source=point.mode,
                    prediction_age_s=point.prediction_age_s,
                    azimuth=azimuth,
                    elevation=elevation,
                    distance=distance,
                    threat_score=threat_score,
                )
                statuses.append(
                    SpecialRidUIStatus(
                        board=board,
                        camera_id=camera_id,
                        target_id=slot.ui_id,
                        azimuth=azimuth,
                        elevation=elevation,
                        distance=distance,
                        threat_score=threat_score,
                        replaced_target_id=replaced_target_id,
                    )
                )
            return statuses


def run_special_rid_ui_sender(
    registry,
    rid_snapshot_provider,
    station_snapshot_provider,
    sender,
    stop_event,
    log_callback=None,
    status_batch_callback=None,
    hz=5.0,
    clock=time.time,
    monotonic=time.monotonic,
):
    """Emit only current special RID snapshots; missed periods are discarded."""

    period_s = 1.0 / max(0.1, float(hz))
    deadline = float(monotonic())
    while not stop_event.is_set():
        now_ts = float(clock())
        try:
            registry.observe_rids(rid_snapshot_provider() or (), now_ts=now_ts)
            station = station_snapshot_provider() or {}
            statuses = registry.ui_statuses(now_ts=now_ts, station=station)
            sent_statuses = []
            for status in statuses:
                sent = sender.send_status(
                    board_str=status.board,
                    camera_id=status.camera_id,
                    target_id=status.target_id,
                    azimuth=status.azimuth,
                    elevation=status.elevation,
                    distance=status.distance,
                    threat_score=status.threat_score,
                    replaced_target_id=status.replaced_target_id,
                )
                if sent:
                    sent_statuses.append(status)
                    registry.ack_ui_status_sent(
                        target_id=status.target_id,
                        replaced_target_id=status.replaced_target_id,
                    )
                if log_callback is not None:
                    try:
                        log_callback({
                            "timestamp": now_ts,
                            "event": "UI_SEND" if sent else "UI_SKIP",
                            "ui_id": status.target_id,
                            "board": status.board,
                            "cam": status.camera_id,
                            "azimuth": status.azimuth,
                            "elevation": status.elevation,
                            "distance": status.distance,
                            "threat_score": status.threat_score,
                            "send_result": 1 if sent else 0,
                            "skip_reason": "" if sent else "udp_send_failed",
                        })
                    except Exception:
                        pass
            if status_batch_callback is not None:
                try:
                    status_batch_callback(tuple(sent_statuses), station, now_ts)
                except Exception as exc:
                    if log_callback is not None:
                        try:
                            log_callback({
                                "timestamp": now_ts,
                                "event": "STRIKE_SEND_SKIP",
                                "send_result": 0,
                                "skip_reason": f"strike_mirror_error:{exc}",
                            })
                        except Exception:
                            pass
        except Exception as exc:
            if status_batch_callback is not None:
                try:
                    status_batch_callback((), {}, now_ts)
                except Exception:
                    pass
            if log_callback is not None:
                try:
                    log_callback({
                        "timestamp": now_ts,
                        "event": "UI_SKIP",
                        "send_result": 0,
                        "skip_reason": f"sender_cycle_error:{exc}",
                    })
                except Exception:
                    pass

        deadline += period_s
        current_mono = float(monotonic())
        if deadline <= current_mono:
            deadline = current_mono + period_s
        if stop_event.wait(max(0.0, deadline - current_mono)):
            break
