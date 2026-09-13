#!/usr/bin/env python3
"""Conservative online calibration and shadow matching for SORT/RID tracks."""

from __future__ import annotations

import json
import math
import statistics
import time
from collections import defaultdict, deque
from pathlib import Path


BLOCKED_COST = 1.0e6


def _linear_sum_assignment(cost_matrix):
    """Pure-Python rectangular Hungarian assignment for small track sets."""
    if not cost_matrix or not cost_matrix[0]:
        return []
    rows = len(cost_matrix)
    cols = len(cost_matrix[0])
    transposed = rows > cols
    matrix = (
        [list(row) for row in zip(*cost_matrix)]
        if transposed else [list(row) for row in cost_matrix]
    )
    n = len(matrix)
    m = len(matrix[0])
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        min_value = [math.inf] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = math.inf
            j1 = 0
            for j in range(1, m + 1):
                if used[j]:
                    continue
                current = matrix[i0 - 1][j - 1] - u[i0] - v[j]
                if current < min_value[j]:
                    min_value[j] = current
                    way[j] = j0
                if min_value[j] < delta:
                    delta = min_value[j]
                    j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    min_value[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    pairs = []
    for column in range(1, m + 1):
        if p[column] == 0:
            continue
        row = p[column] - 1
        col = column - 1
        pairs.append((col, row) if transposed else (row, col))
    return pairs


def _finite(value, default=None):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _percentile(values, quantile):
    values = sorted(float(value) for value in values)
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    position = max(0.0, min(1.0, float(quantile))) * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    ratio = position - lower
    return values[lower] + ratio * (values[upper] - values[lower])


class SensorModel:
    def __init__(self, max_samples=500, az_samples=None, el_samples=None):
        self.max_samples = max(20, int(max_samples))
        self.az_samples = deque(
            (float(value) for value in (az_samples or ())),
            maxlen=self.max_samples,
        )
        self.el_samples = deque(
            (float(value) for value in (el_samples or ())),
            maxlen=self.max_samples,
        )

    @staticmethod
    def _robust(values, sigma_floor, gate_floor, gate_ceiling):
        values = list(values)
        if not values:
            return {
                "n": 0,
                "bias": 0.0,
                "sigma": float(sigma_floor),
                "gate": float(gate_ceiling),
                "p95": math.nan,
            }
        bias = float(statistics.median(values))
        centered = [abs(value - bias) for value in values]
        mad = float(statistics.median(centered))
        sigma = max(float(sigma_floor), 1.4826 * mad)
        p95 = _percentile(centered, 0.95)
        gate = max(float(gate_floor), 3.5 * sigma, 1.25 * p95)
        gate = min(float(gate_ceiling), gate)
        return {
            "n": len(values),
            "bias": bias,
            "sigma": sigma,
            "gate": gate,
            "p95": p95,
        }

    def snapshot(self, min_samples):
        az = self._robust(
            self.az_samples,
            sigma_floor=0.20,
            gate_floor=0.75,
            gate_ceiling=8.0,
        )
        el = self._robust(
            self.el_samples,
            sigma_floor=0.25,
            gate_floor=1.0,
            gate_ceiling=10.0,
        )
        ready = az["n"] >= int(min_samples)
        el_ready = el["n"] >= int(min_samples)
        if not el_ready:
            az_weight, el_weight = 1.0, 0.0
        else:
            az_inverse = 1.0 / max(az["sigma"] ** 2, 1.0e-6)
            el_inverse = 1.0 / max(el["sigma"] ** 2, 1.0e-6)
            raw_el_weight = el_inverse / (az_inverse + el_inverse)
            # Elevation depends on two altitude datums. Until field evidence is
            # strong, it may help association but must never dominate azimuth.
            el_weight = min(0.40, max(0.0, raw_el_weight))
            az_weight = 1.0 - el_weight
        return {
            "ready": ready,
            "el_ready": el_ready,
            "az": az,
            "el": el,
            "az_weight": az_weight,
            "el_weight": el_weight,
        }

    def add(self, az_error, el_error=None):
        az_error = _finite(az_error)
        el_error = _finite(el_error)
        if az_error is not None:
            self.az_samples.append(az_error)
        if el_error is not None:
            self.el_samples.append(el_error)

    def to_json(self):
        return {
            "az_samples": list(self.az_samples),
            "el_samples": list(self.el_samples),
        }


class OnlineMatchLearner:
    """Learn sensor biases and propose identity bindings in shadow mode.

    The learner bootstraps only from one-SORT/one-RID cycles or explicitly
    trusted pairs. Multi-target association is disabled until the relevant
    camera model has enough trusted samples. Only confirmed pairs update the
    model, preventing an ambiguous assignment from reinforcing itself.
    """

    STATE_VERSION = 1

    def __init__(
        self,
        state_path,
        min_samples=12,
        confirm_updates=3,
        bootstrap_max_az_deg=30.0,
        ambiguity_margin=0.20,
        hold_seconds=3.0,
        max_samples=500,
        trusted_pairs=None,
        bootstrap_trajectory_samples=None,
        bootstrap_min_motion_deg=2.0,
        bootstrap_max_shape_p95_deg=2.5,
        bootstrap_max_trend_rmse_deg=1.5,
        bootstrap_history_ttl_s=120.0,
        learned_token_capacity=20000,
    ):
        self.state_path = Path(state_path)
        self.min_samples = max(4, int(min_samples))
        self.confirm_updates = max(1, int(confirm_updates))
        self.bootstrap_max_az_deg = max(1.0, float(bootstrap_max_az_deg))
        self.ambiguity_margin = max(0.0, float(ambiguity_margin))
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.max_samples = max(20, int(max_samples))
        self.trusted_pairs = set(trusted_pairs or ())
        self.bootstrap_trajectory_samples = max(
            4,
            int(
                self.min_samples
                if bootstrap_trajectory_samples is None
                else bootstrap_trajectory_samples
            ),
        )
        self.bootstrap_min_motion_deg = max(
            0.1, float(bootstrap_min_motion_deg)
        )
        self.bootstrap_max_shape_p95_deg = max(
            0.1, float(bootstrap_max_shape_p95_deg)
        )
        self.bootstrap_max_trend_rmse_deg = max(
            0.1, float(bootstrap_max_trend_rmse_deg)
        )
        self.bootstrap_history_ttl_s = max(
            10.0, float(bootstrap_history_ttl_s)
        )
        self.learned_token_capacity = max(1000, int(learned_token_capacity))
        self.models = {}
        self.pending = {}
        self.confirmed = {}
        self.bootstrap_histories = {}
        self.learned_measurements = set()
        self.learned_measurement_order = deque()
        self.dirty = False
        self.last_save_ts = 0.0
        self.load()

    @staticmethod
    def sensor_key(candidate):
        board = str(candidate.get("board", "")).strip() or "unknown_board"
        cam = str(candidate.get("cam", "")).strip() or "unknown_cam"
        logic = str(candidate.get("logic_id", "")).strip() or "unknown_logic"
        return f"{board}/{cam}/logic={logic}"

    @staticmethod
    def pair_key(candidate):
        return int(candidate["sort_track_id"]), str(candidate["rid_id"])

    def model_for(self, sensor_key):
        model = self.models.get(sensor_key)
        if model is None:
            model = SensorModel(max_samples=self.max_samples)
            self.models[sensor_key] = model
        return model

    def load(self):
        if not self.state_path.exists():
            return
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
            if int(payload.get("version", 0)) != self.STATE_VERSION:
                return
            for key, value in payload.get("models", {}).items():
                self.models[str(key)] = SensorModel(
                    max_samples=self.max_samples,
                    az_samples=value.get("az_samples", ()),
                    el_samples=value.get("el_samples", ()),
                )
            print(
                f"[AlignLearn] loaded {len(self.models)} sensor model(s) "
                f"from {self.state_path}"
            )
        except Exception as exc:
            print(f"[AlignLearn][Warn] model load failed: {exc}")

    def save_if_due(self, force=False):
        if not self.dirty:
            return
        now = time.time()
        if not force and now - self.last_save_ts < 5.0:
            return
        payload = {
            "version": self.STATE_VERSION,
            "updated_at": now,
            "min_samples": self.min_samples,
            "models": {
                key: {
                    **model.to_json(),
                    "recommended": model.snapshot(self.min_samples),
                    "method": (
                        "per_camera_robust_bias_mad_inverse_variance_weighting"
                    ),
                }
                for key, model in sorted(self.models.items())
            },
        }
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.state_path.with_suffix(self.state_path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            temporary.replace(self.state_path)
            self.last_save_ts = now
            self.dirty = False
        except Exception as exc:
            print(f"[AlignLearn][Warn] model save failed: {exc}")

    def _evaluate(self, candidate, hold=False):
        sensor_key = self.sensor_key(candidate)
        model = self.model_for(sensor_key)
        snapshot = model.snapshot(self.min_samples)
        az_error = _finite(candidate.get("az_error_aligned_deg"))
        el_error = _finite(candidate.get("el_error_aligned_deg"))
        if az_error is None:
            return None

        if not snapshot["ready"]:
            az_corrected = az_error
            az_gate = self.bootstrap_max_az_deg
            eligible = abs(az_corrected) <= az_gate
            score = abs(az_corrected) / az_gate
            el_corrected = None if el_error is None else el_error
        else:
            az_corrected = az_error - snapshot["az"]["bias"]
            az_gate = snapshot["az"]["gate"] * (1.5 if hold else 1.0)
            eligible = abs(az_corrected) <= az_gate
            az_score = abs(az_corrected) / max(az_gate, 1.0e-6)
            el_corrected = None
            el_score = 0.0
            if snapshot["el_ready"] and el_error is not None:
                el_corrected = el_error - snapshot["el"]["bias"]
                el_gate = snapshot["el"]["gate"] * (1.5 if hold else 1.0)
                eligible = eligible and abs(el_corrected) <= el_gate
                el_score = abs(el_corrected) / max(el_gate, 1.0e-6)
            score = (
                snapshot["az_weight"] * az_score
                + snapshot["el_weight"] * el_score
            )
        if not eligible:
            return None
        return {
            "candidate": candidate,
            "sensor_key": sensor_key,
            "model": snapshot,
            "az_corrected": az_corrected,
            "el_corrected": el_corrected,
            "score": score,
        }

    @staticmethod
    def _measurement_token(candidate):
        sequence = str(candidate.get("rid_measurement_seq", "")).strip()
        if sequence:
            return sequence
        return str(candidate.get("rid_render_timestamp", ""))

    @staticmethod
    def _unwrapped_span(values):
        values = [_finite(value) for value in values]
        values = [value for value in values if value is not None]
        if len(values) < 2:
            return 0.0
        unwrapped = [values[0]]
        for previous, current in zip(values, values[1:]):
            delta = (current - previous + 180.0) % 360.0 - 180.0
            unwrapped.append(unwrapped[-1] + delta)
        return max(unwrapped) - min(unwrapped)

    @staticmethod
    def _linear_span(values):
        values = [_finite(value) for value in values]
        values = [value for value in values if value is not None]
        return 0.0 if len(values) < 2 else max(values) - min(values)

    @staticmethod
    def _shape_metrics(values):
        values = [_finite(value) for value in values]
        values = [value for value in values if value is not None]
        if len(values) < 2:
            return None
        bias = float(statistics.median(values))
        centered = [abs(value - bias) for value in values]
        p95 = _percentile(centered, 0.95)
        deltas = [current - previous for previous, current in zip(values, values[1:])]
        trend_rmse = math.sqrt(
            sum(value * value for value in deltas) / len(deltas)
        )
        return {"bias": bias, "p95": p95, "trend_rmse": trend_rmse}

    def _prune_bootstrap_histories(self, now_ts):
        for key, history in list(self.bootstrap_histories.items()):
            if now_ts - history.get("last_seen_ts", 0.0) > self.bootstrap_history_ttl_s:
                del self.bootstrap_histories[key]

    def _record_bootstrap_histories(self, candidates, now_ts):
        capacity = max(24, self.bootstrap_trajectory_samples * 4)
        for candidate in candidates:
            az_error = _finite(candidate.get("az_error_aligned_deg"))
            if az_error is None or abs(az_error) > self.bootstrap_max_az_deg:
                continue
            key = (*self.pair_key(candidate), self.sensor_key(candidate))
            history = self.bootstrap_histories.get(key)
            if history is None:
                history = {
                    "samples": deque(maxlen=capacity),
                    "tokens": deque(maxlen=capacity),
                    "token_set": set(),
                    "last_seen_ts": now_ts,
                }
                self.bootstrap_histories[key] = history
            history["last_seen_ts"] = now_ts
            token = self._measurement_token(candidate)
            if token in history["token_set"]:
                continue
            if len(history["samples"]) == capacity:
                expired = history["tokens"].popleft()
                history["token_set"].discard(expired)
                history["samples"].popleft()
            history["samples"].append(dict(candidate))
            history["tokens"].append(token)
            history["token_set"].add(token)

    def _bootstrap_trajectory_evaluate(self, candidate):
        sensor_key = self.sensor_key(candidate)
        key = (*self.pair_key(candidate), sensor_key)
        history = self.bootstrap_histories.get(key)
        if history is None:
            return None
        samples = list(history["samples"])
        if len(samples) < self.bootstrap_trajectory_samples:
            return None

        az_motion = self._unwrapped_span(
            sample.get("rid_map_az_deg") for sample in samples
        )
        el_motion = self._linear_span(
            sample.get("rid_el_deg") for sample in samples
        )
        active_metrics = []
        az_metrics = self._shape_metrics(
            sample.get("az_error_aligned_deg") for sample in samples
        )
        el_metrics = self._shape_metrics(
            sample.get("el_error_aligned_deg") for sample in samples
        )
        if az_motion >= self.bootstrap_min_motion_deg and az_metrics is not None:
            active_metrics.append(az_metrics)
        if el_motion >= self.bootstrap_min_motion_deg and el_metrics is not None:
            active_metrics.append(el_metrics)
        if not active_metrics:
            return None
        if any(
            item["p95"] > self.bootstrap_max_shape_p95_deg
            or item["trend_rmse"] > self.bootstrap_max_trend_rmse_deg
            for item in active_metrics
        ):
            return None
        scores = [
            0.70 * item["p95"] / self.bootstrap_max_shape_p95_deg
            + 0.30 * item["trend_rmse"] / self.bootstrap_max_trend_rmse_deg
            for item in active_metrics
        ]
        az_error = _finite(candidate.get("az_error_aligned_deg"))
        el_error = _finite(candidate.get("el_error_aligned_deg"))
        az_bias = az_metrics["bias"] if az_metrics is not None else 0.0
        el_bias = el_metrics["bias"] if el_metrics is not None else 0.0
        return {
            "candidate": candidate,
            "sensor_key": sensor_key,
            "model": self.model_for(sensor_key).snapshot(self.min_samples),
            "az_corrected": None if az_error is None else az_error - az_bias,
            "el_corrected": None if el_error is None else el_error - el_bias,
            "score": sum(scores) / len(scores),
            "bootstrap_shape": True,
            "bootstrap_samples": len(samples),
            "bootstrap_az_motion_deg": az_motion,
            "bootstrap_el_motion_deg": el_motion,
        }

    def _record_pending(self, evaluated):
        candidate = evaluated["candidate"]
        sort_id, rid_id = self.pair_key(candidate)
        token = self._measurement_token(candidate)
        pending = self.pending.get(sort_id)
        if pending is None or pending["rid_id"] != rid_id:
            pending = {
                "rid_id": rid_id,
                "tokens": set(),
                "samples": [],
            }
            self.pending[sort_id] = pending
        if token not in pending["tokens"]:
            pending["tokens"].add(token)
            pending["samples"].append(candidate)
        return pending

    def _learn_candidate(self, candidate):
        sensor_key = self.sensor_key(candidate)
        token = (
            sensor_key,
            str(candidate.get("rid_id", "")),
            self._measurement_token(candidate),
        )
        if token in self.learned_measurements:
            return False
        if len(self.learned_measurement_order) >= self.learned_token_capacity:
            expired = self.learned_measurement_order.popleft()
            self.learned_measurements.discard(expired)
        self.learned_measurements.add(token)
        self.learned_measurement_order.append(token)
        self.model_for(sensor_key).add(
            candidate.get("az_error_aligned_deg"),
            candidate.get("el_error_aligned_deg"),
        )
        self.dirty = True
        return True

    def _confirm(self, evaluated, now_ts):
        candidate = evaluated["candidate"]
        sort_id, rid_id = self.pair_key(candidate)
        pending = self._record_pending(evaluated)
        hits = len(pending["tokens"])
        if hits < self.confirm_updates:
            return f"candidate_{hits}/{self.confirm_updates}", False
        self.confirmed[sort_id] = {
            "rid_id": rid_id,
            "last_good_ts": now_ts,
            "sensor_key": evaluated["sensor_key"],
        }
        for sample in pending["samples"]:
            self._learn_candidate(sample)
        self.pending.pop(sort_id, None)
        return "confirmed_now", True

    def _decorate(self, evaluated, state):
        candidate = dict(evaluated["candidate"])
        snapshot = self.model_for(evaluated["sensor_key"]).snapshot(
            self.min_samples
        )
        candidate.update({
            "learning_state": state,
            "sensor_key": evaluated["sensor_key"],
            "match_score": evaluated["score"],
            "az_corrected_deg": evaluated["az_corrected"],
            "el_corrected_deg": evaluated["el_corrected"],
            "learned_sample_count": snapshot["az"]["n"],
            "learned_ready": int(snapshot["ready"]),
            "learned_az_bias_deg": snapshot["az"]["bias"],
            "learned_az_sigma_deg": snapshot["az"]["sigma"],
            "learned_az_gate_deg": snapshot["az"]["gate"],
            "learned_el_bias_deg": snapshot["el"]["bias"],
            "learned_el_sigma_deg": snapshot["el"]["sigma"],
            "learned_el_gate_deg": snapshot["el"]["gate"],
            "learned_az_weight": snapshot["az_weight"],
            "learned_el_weight": snapshot["el_weight"],
            "bootstrap_shape": int(bool(evaluated.get("bootstrap_shape"))),
            "bootstrap_samples": evaluated.get("bootstrap_samples", ""),
            "bootstrap_az_motion_deg": evaluated.get(
                "bootstrap_az_motion_deg", ""
            ),
            "bootstrap_el_motion_deg": evaluated.get(
                "bootstrap_el_motion_deg", ""
            ),
        })
        return candidate

    def process_cycle(self, candidates, now_ts=None):
        now_ts = time.time() if now_ts is None else float(now_ts)
        self._prune_bootstrap_histories(now_ts)
        candidates = [dict(candidate) for candidate in candidates]
        by_pair = {self.pair_key(candidate): candidate for candidate in candidates}
        sort_ids = {pair[0] for pair in by_pair}
        rid_ids = {pair[1] for pair in by_pair}
        selected = []
        reserved_sort = set()
        reserved_rid = set()

        # Preserve already confirmed identities before considering new pairs.
        for sort_id, state in list(self.confirmed.items()):
            rid_id = state["rid_id"]
            candidate = by_pair.get((sort_id, rid_id))
            hold_alive = now_ts - state["last_good_ts"] <= self.hold_seconds
            if candidate is None:
                if not hold_alive:
                    del self.confirmed[sort_id]
                    continue
                if sort_id in sort_ids:
                    reserved_sort.add(sort_id)
                if rid_id in rid_ids:
                    reserved_rid.add(rid_id)
                continue
            evaluated = self._evaluate(candidate, hold=True)
            if evaluated is None:
                if not hold_alive:
                    del self.confirmed[sort_id]
                    continue
                reserved_sort.add(sort_id)
                reserved_rid.add(rid_id)
                continue
            state["last_good_ts"] = now_ts
            self._learn_candidate(candidate)
            selected.append(self._decorate(evaluated, "confirmed"))
            reserved_sort.add(sort_id)
            reserved_rid.add(rid_id)

        remaining = [
            candidate for candidate in candidates
            if int(candidate["sort_track_id"]) not in reserved_sort
            and str(candidate["rid_id"]) not in reserved_rid
        ]

        # Known physical pairs, when configured, are trusted bootstrap input.
        trusted = [
            candidate for candidate in remaining
            if self.pair_key(candidate) in self.trusted_pairs
        ]
        if trusted:
            for candidate in trusted:
                evaluated = self._evaluate(candidate)
                if evaluated is None:
                    continue
                state, _ = self._confirm(evaluated, now_ts)
                selected.append(self._decorate(evaluated, f"trusted_{state}"))
            self.save_if_due()
            return selected

        # One-to-one scenes are the only unlabeled bootstrap source.
        remaining_sort = {int(item["sort_track_id"]) for item in remaining}
        remaining_rid = {str(item["rid_id"]) for item in remaining}
        if len(remaining_sort) == 1 and len(remaining_rid) == 1 and remaining:
            evaluated = self._evaluate(remaining[0])
            if evaluated is not None:
                state, _ = self._confirm(evaluated, now_ts)
                selected.append(self._decorate(evaluated, f"single_{state}"))
            self.save_if_due()
            return selected

        # With false alarms present, an uncalibrated one-RID/multi-SORT scene
        # bootstraps from trajectory shape: the true pair keeps SORT-RID
        # residual nearly constant while a false track diverges as the UAV
        # moves. Stationary/ambiguous scenes intentionally produce no match.
        self._record_bootstrap_histories(remaining, now_ts)
        evaluated_candidates = []
        for candidate in remaining:
            sensor_key = self.sensor_key(candidate)
            model_ready = self.model_for(sensor_key).snapshot(
                self.min_samples
            )["ready"]
            evaluated = (
                self._evaluate(candidate)
                if model_ready
                else self._bootstrap_trajectory_evaluate(candidate)
            )
            if evaluated is not None:
                evaluated_candidates.append(evaluated)

        sort_order = sorted({
            int(item["candidate"]["sort_track_id"])
            for item in evaluated_candidates
        })
        rid_order = sorted({
            str(item["candidate"]["rid_id"])
            for item in evaluated_candidates
        })
        sort_index = {value: index for index, value in enumerate(sort_order)}
        rid_index = {value: index for index, value in enumerate(rid_order)}
        evaluated_by_pair = {}
        cost_matrix = [
            [BLOCKED_COST for _ in rid_order] for _ in sort_order
        ]
        for evaluated in evaluated_candidates:
            sort_id, rid_id = self.pair_key(evaluated["candidate"])
            row = sort_index[sort_id]
            col = rid_index[rid_id]
            cost_matrix[row][col] = evaluated["score"]
            evaluated_by_pair[(row, col)] = evaluated

        accepted = []
        for row, col in _linear_sum_assignment(cost_matrix):
            score = cost_matrix[row][col]
            evaluated = evaluated_by_pair.get((row, col))
            if score >= BLOCKED_COST or evaluated is None:
                continue
            row_scores = sorted(
                value for value in cost_matrix[row] if value < BLOCKED_COST
            )
            col_scores = sorted(
                cost_matrix[index][col]
                for index in range(len(sort_order))
                if cost_matrix[index][col] < BLOCKED_COST
            )
            row_margin = math.inf if len(row_scores) < 2 else row_scores[1] - score
            col_margin = math.inf if len(col_scores) < 2 else col_scores[1] - score
            if (
                row_margin < self.ambiguity_margin
                or col_margin < self.ambiguity_margin
            ):
                continue
            accepted.append(evaluated)

        used_sort = set()
        used_rid = set()
        for evaluated in sorted(accepted, key=lambda item: item["score"]):
            sort_id, rid_id = self.pair_key(evaluated["candidate"])
            if sort_id in used_sort or rid_id in used_rid:
                continue
            state, confirmed_now = self._confirm(evaluated, now_ts)
            if confirmed_now:
                self._learn_candidate(evaluated["candidate"])
            selected.append(self._decorate(evaluated, f"multi_{state}"))
            used_sort.add(sort_id)
            used_rid.add(rid_id)

        for sort_id in list(self.pending):
            if sort_id not in used_sort and sort_id not in reserved_sort:
                self.pending.pop(sort_id, None)
        self.save_if_due()
        return selected

    def model_summaries(self):
        summaries = []
        for sensor_key, model in sorted(self.models.items()):
            value = model.snapshot(self.min_samples)
            summaries.append({"sensor_key": sensor_key, **value})
        return summaries

    def close(self):
        self.save_if_due(force=True)
