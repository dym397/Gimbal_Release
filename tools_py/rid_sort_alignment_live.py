#!/usr/bin/env python3
"""Live/offline SORT-to-RID angle alignment diagnostics.

This tool is deliberately read-only with respect to the tracking runtime.  It
tails the production ``track_summary_*.csv`` and ``rid_association_*.csv``
files, interpolates each SORT trajectory to the RID render timestamp, and
records the resulting azimuth/elevation residuals.

The residual convention is always::

    residual = SORT - RID

Positive azimuth residual means the SORT map azimuth is clockwise/eastward of
RID.  Positive elevation residual means SORT is above RID.

For an unambiguous calibration, test one physical target at a time.  With
multiple targets, pass explicit ``--pair SORT_ID:RID_ID`` arguments.  The tool
does not infer a supposedly true identity from the same angle error that is
being measured unless ``--pair-mode logged`` is explicitly requested.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
import tempfile
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

from rid_sort_online_learner import OnlineMatchLearner


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = PROJECT_ROOT / "logs"
DEFAULT_LEARNER_STATE = PROJECT_ROOT / "calibration" / "rid_sort_alignment_model.json"


OUTPUT_FIELDS = (
    "analysis_timestamp",
    "cycle",
    "pair_source",
    "sort_track_id",
    "board",
    "cam",
    "logic_id",
    "rid_ui_id",
    "rid_id",
    "rid_measurement_seq",
    "rid_render_mode",
    "rid_render_timestamp",
    "cycle_timestamp",
    "rid_render_delay_s",
    "sort_alignment_mode",
    "sort_alignment_gap_s",
    "sort_relative_az_aligned_deg",
    "sort_map_az_aligned_deg",
    "sort_el_aligned_deg",
    "sort_map_az_current_deg",
    "sort_el_current_deg",
    "rid_map_az_deg",
    "rid_el_deg",
    "az_error_unaligned_deg",
    "az_error_aligned_deg",
    "el_error_unaligned_deg",
    "el_error_aligned_deg",
    "distance_m",
    "rid_age_s",
    "device_heading_deg",
    "learning_state",
    "sensor_key",
    "match_score",
    "az_corrected_deg",
    "el_corrected_deg",
    "learned_sample_count",
    "learned_ready",
    "learned_az_bias_deg",
    "learned_az_sigma_deg",
    "learned_az_gate_deg",
    "learned_el_bias_deg",
    "learned_el_sigma_deg",
    "learned_el_gate_deg",
    "learned_az_weight",
    "learned_el_weight",
    "bootstrap_shape",
    "bootstrap_samples",
    "bootstrap_az_motion_deg",
    "bootstrap_el_motion_deg",
)


def finite_float(value, default=None):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def integer(value, default=None):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def signed_circular_error_deg(a, b):
    """Return signed ``a - b`` in [-180, 180)."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def interpolate_angle_deg(before, after, ratio):
    delta = signed_circular_error_deg(after, before)
    return (float(before) + float(ratio) * delta) % 360.0


def percentile(values, quantile):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, float(quantile))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] + ratio * (ordered[upper] - ordered[lower])


@dataclass
class SortSample:
    timestamp: float
    relative_az: float
    elevation: float


@dataclass
class InterpolatedSort:
    timestamp: float
    relative_az: float
    elevation: float
    mode: str
    gap_s: float


class SortHistory:
    def __init__(self, history_seconds=60.0):
        self.history_seconds = max(2.0, float(history_seconds))
        self.samples = defaultdict(list)

    def add(self, track_id, sample):
        track_id = int(track_id)
        history = self.samples[track_id]
        if history and abs(history[-1].timestamp - sample.timestamp) < 1.0e-9:
            history[-1] = sample
        elif not history or sample.timestamp > history[-1].timestamp:
            history.append(sample)
        else:
            timestamps = [item.timestamp for item in history]
            history.insert(bisect.bisect_left(timestamps, sample.timestamp), sample)

        cutoff = sample.timestamp - self.history_seconds
        while len(history) > 2 and history[1].timestamp < cutoff:
            history.pop(0)

    def interpolate(self, track_id, timestamp, tolerance_s):
        history = self.samples.get(int(track_id), ())
        if not history:
            return None
        timestamp = float(timestamp)
        tolerance_s = max(0.0, float(tolerance_s))
        times = [item.timestamp for item in history]
        index = bisect.bisect_left(times, timestamp)

        if index < len(history) and abs(history[index].timestamp - timestamp) < 1.0e-9:
            sample = history[index]
            return InterpolatedSort(
                timestamp, sample.relative_az, sample.elevation, "exact", 0.0
            )

        if 0 < index < len(history):
            before = history[index - 1]
            after = history[index]
            left_gap = timestamp - before.timestamp
            right_gap = after.timestamp - timestamp
            max_gap = max(left_gap, right_gap)
            if max_gap > tolerance_s:
                return None
            span = after.timestamp - before.timestamp
            ratio = 0.0 if span <= 1.0e-9 else left_gap / span
            return InterpolatedSort(
                timestamp=timestamp,
                relative_az=interpolate_angle_deg(
                    before.relative_az, after.relative_az, ratio
                ),
                elevation=(
                    before.elevation
                    + ratio * (after.elevation - before.elevation)
                ),
                mode="interpolated",
                gap_s=max_gap,
            )

        nearest = history[0] if index == 0 else history[-1]
        gap = abs(nearest.timestamp - timestamp)
        if gap > tolerance_s:
            return None
        return InterpolatedSort(
            timestamp,
            nearest.relative_az,
            nearest.elevation,
            "nearest",
            gap,
        )


def parse_id_set(value):
    values = set()
    for item in str(value or "").split(";"):
        parsed = integer(item.strip())
        if parsed is not None:
            values.add(parsed)
    return values


def parse_track_states(row, allowed_ids=None):
    timestamp = finite_float(row.get("timestamp"))
    if timestamp is None:
        return []
    parsed = []
    for item in str(row.get("track_states", "")).split(";"):
        item = item.strip()
        if not item or ":" not in item:
            continue
        track_text, state_text = item.split(":", 1)
        values = state_text.split(",")
        if len(values) < 2:
            continue
        track_id = integer(track_text)
        relative_az = finite_float(values[0])
        elevation = finite_float(values[1])
        if track_id is None or relative_az is None or elevation is None:
            continue
        if allowed_ids is not None and track_id not in allowed_ids:
            continue
        parsed.append(
            (
                track_id,
                SortSample(timestamp, relative_az % 360.0, elevation),
            )
        )
    return parsed


class ResidualStats:
    def __init__(self, rolling_window=60):
        self.n = 0
        self.sum_value = 0.0
        self.sum_abs = 0.0
        self.sum_sq = 0.0
        self.max_abs = 0.0
        self.recent = deque(maxlen=max(2, int(rolling_window)))

    def add(self, value):
        if value is None or not math.isfinite(float(value)):
            return
        value = float(value)
        self.n += 1
        self.sum_value += value
        self.sum_abs += abs(value)
        self.sum_sq += value * value
        self.max_abs = max(self.max_abs, abs(value))
        self.recent.append(value)

    def snapshot(self):
        if self.n == 0:
            return {
                "n": 0,
                "bias": math.nan,
                "mae": math.nan,
                "rmse": math.nan,
                "max_abs": math.nan,
                "rolling_bias": math.nan,
                "rolling_mae": math.nan,
                "rolling_p95_abs": math.nan,
            }
        recent = list(self.recent)
        return {
            "n": self.n,
            "bias": self.sum_value / self.n,
            "mae": self.sum_abs / self.n,
            "rmse": math.sqrt(self.sum_sq / self.n),
            "max_abs": self.max_abs,
            "rolling_bias": sum(recent) / len(recent),
            "rolling_mae": sum(abs(value) for value in recent) / len(recent),
            "rolling_p95_abs": percentile(
                [abs(value) for value in recent], 0.95
            ),
        }


class OutputLog:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(
            self.file, fieldnames=OUTPUT_FIELDS, extrasaction="ignore"
        )
        self.writer.writeheader()
        self.file.flush()

    def write(self, row):
        self.writer.writerow({field: row.get(field, "") for field in OUTPUT_FIELDS})
        self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()


class CsvFollower:
    def __init__(self, path, from_start=False, wait_for_header=False):
        self.path = Path(path)
        self.file = self.path.open("r", encoding="utf-8-sig", newline="")
        while True:
            self.file.seek(0)
            header_line = self.file.readline()
            if header_line.endswith(("\n", "\r")):
                break
            if not wait_for_header:
                self.file.close()
                raise ValueError(f"CSV has no header: {self.path}")
            time.sleep(0.05)
        self.fields = next(csv.reader([header_line]))
        if not from_start:
            self.file.seek(0, 2)

    def read_available(self):
        rows = []
        while True:
            position = self.file.tell()
            line = self.file.readline()
            if not line:
                break
            if not line.endswith(("\n", "\r")):
                self.file.seek(position)
                break
            values = next(csv.reader([line]))
            if len(values) < len(self.fields):
                values.extend([""] * (len(self.fields) - len(values)))
            rows.append(dict(zip(self.fields, values)))
        return rows

    def close(self):
        self.file.close()


class AlignmentAnalyzer:
    def __init__(self, args, output):
        self.args = args
        self.output = output
        self.history = SortHistory(args.history_seconds)
        self.current_cycle = None
        self.az_stats = defaultdict(
            lambda: ResidualStats(args.rolling_window)
        )
        self.el_stats = defaultdict(
            lambda: ResidualStats(args.rolling_window)
        )
        self.last_print = defaultdict(float)
        self.skipped_alignment = 0
        self.skipped_pairing = 0
        self.warned_multi = False
        self.warned_legacy_summary = False
        self.learner = None
        if args.pair_mode == "learned":
            self.learner = OnlineMatchLearner(
                state_path=args.learner_state,
                min_samples=args.learner_min_samples,
                confirm_updates=args.learner_confirm_updates,
                bootstrap_max_az_deg=args.learner_bootstrap_max_az,
                ambiguity_margin=args.learner_ambiguity_margin,
                hold_seconds=args.learner_hold_seconds,
                trusted_pairs=args.pairs,
                bootstrap_trajectory_samples=(
                    args.learner_bootstrap_trajectory_samples
                ),
                bootstrap_min_motion_deg=args.learner_bootstrap_min_motion,
                bootstrap_max_shape_p95_deg=(
                    args.learner_bootstrap_max_shape_p95
                ),
                bootstrap_max_trend_rmse_deg=(
                    args.learner_bootstrap_max_trend_rmse
                ),
            )

    def consume_summary(self, row):
        if "ui_ids" in row:
            allowed_ids = parse_id_set(row.get("ui_ids"))
        else:
            # Old captures predate the explicit UI-confirmed ID column. Keep
            # offline compatibility, but make the weaker semantic visible.
            allowed_ids = parse_id_set(row.get("valid_ids"))
            if not self.warned_legacy_summary:
                self.warned_legacy_summary = True
                print(
                    "[Align][Warn] track_summary has no ui_ids column; "
                    "falling back to internally confirmed valid_ids.",
                    file=sys.stderr,
                )
        for track_id, sample in parse_track_states(row, allowed_ids):
            self.history.add(track_id, sample)

    def consume_association(self, row):
        event = row.get("event", "")
        if event == "RID_UI_FUSION_SUMMARY":
            self.flush_cycle()
            self.current_cycle = {
                "summary": row,
                "sort": {},
                "rid": {},
                "logged_pairs": [],
            }
            return
        if self.current_cycle is None:
            return
        cycle = str(row.get("cycle", ""))
        expected = str(self.current_cycle["summary"].get("cycle", ""))
        if cycle and expected and cycle != expected:
            return
        if event == "SORT_TRACK":
            track_id = integer(row.get("sort_track_id"))
            if track_id is not None:
                self.current_cycle["sort"][track_id] = row
        elif event == "RID_TRACK":
            rid_key = self._rid_key(row)
            if rid_key is not None:
                self.current_cycle["rid"][rid_key] = row
        elif event == "RID_UI_STATELESS_PAIR":
            self.current_cycle["logged_pairs"].append(row)

    @staticmethod
    def _rid_key(row):
        rid_id = str(row.get("rid_id", "")).strip()
        if rid_id:
            return rid_id
        rid_ui_id = integer(row.get("rid_ui_id"))
        return None if rid_ui_id is None else f"ui:{rid_ui_id}"

    def _explicit_pairs(self, sort_rows, rid_rows):
        selected = []
        for sort_id, rid_selector in self.args.pairs:
            sort_row = sort_rows.get(sort_id)
            rid_row = rid_rows.get(rid_selector)
            if rid_row is None and rid_selector.startswith("ui:"):
                wanted = integer(rid_selector[3:])
                rid_row = next(
                    (
                        row for row in rid_rows.values()
                        if integer(row.get("rid_ui_id")) == wanted
                    ),
                    None,
                )
            if sort_row is not None and rid_row is not None:
                selected.append((sort_row, rid_row, "explicit"))
        return selected

    def _select_pairs(self, cycle):
        sort_rows = cycle["sort"]
        rid_rows = cycle["rid"]
        if self.args.pairs:
            return self._explicit_pairs(sort_rows, rid_rows)
        if self.args.pair_mode == "all":
            return [
                (sort_row, rid_row, "all_candidates")
                for sort_row in sort_rows.values()
                for rid_row in rid_rows.values()
            ]
        if self.args.pair_mode == "logged":
            selected = []
            for pair in cycle["logged_pairs"]:
                sort_id = integer(pair.get("sort_track_id"))
                rid_key = self._rid_key(pair)
                sort_row = sort_rows.get(sort_id)
                rid_row = rid_rows.get(rid_key)
                if sort_row is not None and rid_row is not None:
                    selected.append((sort_row, rid_row, "stateless_provisional"))
            return selected
        if len(sort_rows) == 1 and len(rid_rows) == 1:
            return [
                (
                    next(iter(sort_rows.values())),
                    next(iter(rid_rows.values())),
                    "single_target",
                )
            ]
        if sort_rows and rid_rows and not self.warned_multi:
            self.warned_multi = True
            print(
                "[Align][Warn] Multiple SORT/RID targets detected. No identity "
                "is assumed in auto mode; use --pair SORT_ID:RID_ID, "
                "--pair-mode logged, or --pair-mode all.",
                file=sys.stderr,
            )
        self.skipped_pairing += 1
        return []

    def _analyze_pair(self, cycle, sort_row, rid_row, pair_source, emit=True):
        sort_id = integer(sort_row.get("sort_track_id"))
        rid_id = self._rid_key(rid_row)
        rid_az = finite_float(rid_row.get("rid_map_az"))
        if sort_id is None or rid_id is None or rid_az is None:
            return None

        cycle_ts = finite_float(rid_row.get("timestamp"))
        if cycle_ts is None:
            cycle_ts = finite_float(cycle["summary"].get("timestamp"))
        rid_ts = finite_float(rid_row.get("rid_render_timestamp"), cycle_ts)
        if cycle_ts is None or rid_ts is None:
            return None

        aligned = self.history.interpolate(
            sort_id, rid_ts, self.args.sync_tolerance
        )
        if aligned is None:
            self.skipped_alignment += 1
            return None

        heading = finite_float(rid_row.get("device_heading_deg"), 0.0)
        sort_map_aligned = (aligned.relative_az + heading) % 360.0
        sort_map_current = finite_float(sort_row.get("sort_map_az"))
        sort_el_current = finite_float(sort_row.get("sort_el"))
        rid_el = finite_float(rid_row.get("rid_elevation_deg"))
        az_aligned = signed_circular_error_deg(sort_map_aligned, rid_az)
        az_unaligned = (
            None if sort_map_current is None
            else signed_circular_error_deg(sort_map_current, rid_az)
        )
        el_aligned = (
            None if rid_el is None else aligned.elevation - rid_el
        )
        el_unaligned = (
            None if rid_el is None or sort_el_current is None
            else sort_el_current - rid_el
        )

        row = {
            "analysis_timestamp": f"{time.time():.6f}",
            "cycle": cycle["summary"].get("cycle", ""),
            "pair_source": pair_source,
            "sort_track_id": sort_id,
            "board": sort_row.get("board", ""),
            "cam": sort_row.get("cam", ""),
            "logic_id": sort_row.get("logic_id", ""),
            "rid_ui_id": rid_row.get("rid_ui_id", ""),
            "rid_id": rid_row.get("rid_id") or rid_id,
            "rid_measurement_seq": rid_row.get("rid_measurement_seq", ""),
            "rid_render_mode": rid_row.get("rid_render_mode", ""),
            "rid_render_timestamp": f"{rid_ts:.6f}",
            "cycle_timestamp": f"{cycle_ts:.6f}",
            "rid_render_delay_s": f"{cycle_ts - rid_ts:.6f}",
            "sort_alignment_mode": aligned.mode,
            "sort_alignment_gap_s": f"{aligned.gap_s:.6f}",
            "sort_relative_az_aligned_deg": f"{aligned.relative_az:.6f}",
            "sort_map_az_aligned_deg": f"{sort_map_aligned:.6f}",
            "sort_el_aligned_deg": f"{aligned.elevation:.6f}",
            "sort_map_az_current_deg": (
                "" if sort_map_current is None else f"{sort_map_current:.6f}"
            ),
            "sort_el_current_deg": (
                "" if sort_el_current is None else f"{sort_el_current:.6f}"
            ),
            "rid_map_az_deg": f"{rid_az:.6f}",
            "rid_el_deg": "" if rid_el is None else f"{rid_el:.6f}",
            "az_error_unaligned_deg": (
                "" if az_unaligned is None else f"{az_unaligned:.6f}"
            ),
            "az_error_aligned_deg": f"{az_aligned:.6f}",
            "el_error_unaligned_deg": (
                "" if el_unaligned is None else f"{el_unaligned:.6f}"
            ),
            "el_error_aligned_deg": (
                "" if el_aligned is None else f"{el_aligned:.6f}"
            ),
            "distance_m": rid_row.get("distance_m", ""),
            "rid_age_s": rid_row.get("rid_age_s", ""),
            "device_heading_deg": f"{heading:.6f}",
        }
        if emit:
            self._emit_row(row)
        return row

    def _emit_row(self, row):
        sort_id = integer(row.get("sort_track_id"))
        rid_id = str(row.get("rid_id", ""))
        if sort_id is None or not rid_id:
            return
        key = (sort_id, rid_id)
        self.az_stats[key].add(finite_float(row.get("az_error_aligned_deg")))
        self.el_stats[key].add(finite_float(row.get("el_error_aligned_deg")))
        self.output.write(row)
        self._maybe_print(key, row)

    def _maybe_print(self, key, row):
        now = time.time()
        if now - self.last_print[key] < self.args.print_interval:
            return
        self.last_print[key] = now
        az = self.az_stats[key].snapshot()
        el = self.el_stats[key].snapshot()
        unaligned = row["az_error_unaligned_deg"] or "nan"
        el_text = (
            "el=unavailable"
            if el["n"] == 0
            else (
                f"el_now={float(row['el_error_aligned_deg']):+.3f}deg "
                f"el_bias={el['rolling_bias']:+.3f}deg "
                f"el_p95={el['rolling_p95_abs']:.3f}deg"
            )
        )
        print(
            f"[Align] S{key[0]} <-> RID[{key[1]}] "
            f"n={az['n']} delay={float(row['rid_render_delay_s']):.3f}s "
            f"az_unaligned={float(unaligned):+.3f}deg "
            f"az_aligned={float(row['az_error_aligned_deg']):+.3f}deg "
            f"az_bias={az['rolling_bias']:+.3f}deg "
            f"az_mae={az['rolling_mae']:.3f}deg "
            f"az_p95={az['rolling_p95_abs']:.3f}deg {el_text}"
        )
        if row.get("learning_state"):
            bootstrap_text = ""
            if int(row.get("bootstrap_shape", 0) or 0):
                bootstrap_text = (
                    f" bootstrap_samples={int(row.get('bootstrap_samples', 0))}"
                    f" motion=(az={float(row.get('bootstrap_az_motion_deg', 0.0)):.2f},"
                    f"el={float(row.get('bootstrap_el_motion_deg', 0.0)):.2f})deg"
                )
            print(
                f"[AlignLearn] sensor={row.get('sensor_key', '')} "
                f"pair=S{key[0]}<->RID[{key[1]}] "
                f"state={row.get('learning_state', '')} "
                f"score={float(row.get('match_score', math.nan)):.3f} "
                f"samples={int(row.get('learned_sample_count', 0))} "
                f"ready={int(row.get('learned_ready', 0))} "
                f"az_bias={float(row.get('learned_az_bias_deg', math.nan)):+.3f}deg "
                f"az_sigma={float(row.get('learned_az_sigma_deg', math.nan)):.3f}deg "
                f"az_gate={float(row.get('learned_az_gate_deg', math.nan)):.3f}deg "
                f"el_bias={float(row.get('learned_el_bias_deg', math.nan)):+.3f}deg "
                f"weights=({float(row.get('learned_az_weight', math.nan)):.3f},"
                f"{float(row.get('learned_el_weight', math.nan)):.3f})"
                f"{bootstrap_text}"
            )

    def flush_cycle(self):
        if self.current_cycle is None:
            return
        if self.learner is not None:
            candidates = []
            for sort_row in self.current_cycle["sort"].values():
                for rid_row in self.current_cycle["rid"].values():
                    candidate = self._analyze_pair(
                        self.current_cycle,
                        sort_row,
                        rid_row,
                        "learned_candidate",
                        emit=False,
                    )
                    if candidate is not None:
                        candidates.append(candidate)
            cycle_ts = finite_float(
                self.current_cycle["summary"].get("timestamp"), time.time()
            )
            for learned in self.learner.process_cycle(candidates, cycle_ts):
                learned["pair_source"] = "learned_shadow"
                self._emit_row(learned)
            self.current_cycle = None
            return
        for sort_row, rid_row, source in self._select_pairs(self.current_cycle):
            self._analyze_pair(self.current_cycle, sort_row, rid_row, source)
        self.current_cycle = None

    def print_final_summary(self):
        self.flush_cycle()
        print("\n[Align] Final summary (residual = SORT - RID)")
        if not self.az_stats:
            print("  no aligned samples produced")
        for key in sorted(self.az_stats, key=lambda item: (item[0], item[1])):
            az = self.az_stats[key].snapshot()
            el = self.el_stats[key].snapshot()
            el_text = (
                "el=unavailable"
                if el["n"] == 0
                else (
                    f"el_n={el['n']} el_bias={el['bias']:+.4f}deg "
                    f"el_mae={el['mae']:.4f}deg "
                    f"el_rmse={el['rmse']:.4f}deg "
                    f"el_max={el['max_abs']:.4f}deg"
                )
            )
            print(
                f"  S{key[0]} <-> RID[{key[1]}]: "
                f"az_n={az['n']} az_bias={az['bias']:+.4f}deg "
                f"az_mae={az['mae']:.4f}deg "
                f"az_rmse={az['rmse']:.4f}deg "
                f"az_max={az['max_abs']:.4f}deg; {el_text}"
            )
        print(
            f"  skipped_no_time_alignment={self.skipped_alignment}, "
            f"skipped_ambiguous_auto_cycles={self.skipped_pairing}"
        )
        if self.learner is not None:
            print("[AlignLearn] Learned sensor models")
            for summary in self.learner.model_summaries():
                print(
                    f"  {summary['sensor_key']}: "
                    f"n={summary['az']['n']} ready={int(summary['ready'])} "
                    f"az_bias={summary['az']['bias']:+.4f}deg "
                    f"az_sigma={summary['az']['sigma']:.4f}deg "
                    f"az_gate={summary['az']['gate']:.4f}deg "
                    f"el_bias={summary['el']['bias']:+.4f}deg "
                    f"weights=({summary['az_weight']:.3f},"
                    f"{summary['el_weight']:.3f})"
                )
            self.learner.close()


def parse_pair(value):
    if ":" not in value:
        raise argparse.ArgumentTypeError(
            "pair must be SORT_ID:RID_ID or SORT_ID:ui:RID_UI_ID"
        )
    sort_text, rid_selector = value.split(":", 1)
    sort_id = integer(sort_text)
    rid_selector = rid_selector.strip()
    if sort_id is None or not rid_selector:
        raise argparse.ArgumentTypeError(f"invalid pair: {value}")
    return sort_id, rid_selector


def matching_summary_path(association_path):
    name = association_path.name
    prefix = "rid_association_"
    if not name.startswith(prefix):
        raise ValueError(
            f"association filename must start with {prefix!r}: {association_path}"
        )
    suffix = name[len(prefix):]
    return association_path.with_name(f"track_summary_{suffix}")


def find_latest_log_pair(log_dir, not_before=0.0):
    not_before = max(0.0, float(not_before))
    candidates = sorted(
        Path(log_dir).glob("**/rid_association_*.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for association in candidates:
        summary = matching_summary_path(association)
        if (
            summary.exists()
            and association.stat().st_mtime >= not_before
            and summary.stat().st_mtime >= not_before
        ):
            return summary, association
    return None


def resolve_log_pair(args):
    if args.association_file:
        association = Path(args.association_file).resolve()
        summary = (
            Path(args.summary_file).resolve()
            if args.summary_file
            else matching_summary_path(association)
        )
        if not association.exists():
            raise FileNotFoundError(association)
        if not summary.exists():
            raise FileNotFoundError(summary)
        return summary, association

    deadline = None if args.wait_for_logs <= 0 else time.time() + args.wait_for_logs
    while True:
        result = find_latest_log_pair(args.logs_dir, args.log_not_before)
        if result is not None:
            return result
        if deadline is not None and time.time() >= deadline:
            raise FileNotFoundError(
                f"no matching track_summary/rid_association CSV pair under "
                f"{Path(args.logs_dir).resolve()}"
            )
        print("[Align] Waiting for production log files...", file=sys.stderr)
        time.sleep(1.0)


def load_csv(path):
    with Path(path).open("r", encoding="utf-8-sig", newline="") as file_obj:
        return list(csv.DictReader(file_obj))


def timestamped_events(summary_rows, association_rows):
    events = []
    order = 0
    for row in summary_rows:
        timestamp = finite_float(row.get("timestamp"))
        if timestamp is not None:
            events.append((timestamp, 0, order, "summary", row))
            order += 1
    for row in association_rows:
        timestamp = finite_float(row.get("timestamp"))
        if timestamp is not None:
            events.append((timestamp, 1, order, "association", row))
            order += 1
    return sorted(events)


def process_events(analyzer, events):
    for _, _, _, source, row in events:
        if source == "summary":
            analyzer.consume_summary(row)
        else:
            analyzer.consume_association(row)


def default_output_path(association_path):
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return association_path.parent / f"rid_sort_alignment_{stamp}.csv"


def run_offline(args, summary_path, association_path, analyzer):
    process_events(
        analyzer,
        timestamped_events(load_csv(summary_path), load_csv(association_path)),
    )


def run_follow(args, summary_path, association_path, analyzer):
    if args.from_start:
        run_offline(args, summary_path, association_path, analyzer)
        analyzer.flush_cycle()
    # The producer creates all log files before buffered headers necessarily
    # become visible. Live mode must tolerate that short startup window.
    summary_tail = CsvFollower(
        summary_path, from_start=False, wait_for_header=True
    )
    association_tail = CsvFollower(
        association_path, from_start=False, wait_for_header=True
    )
    started = time.time()
    try:
        while args.duration <= 0 or time.time() - started < args.duration:
            summary_rows = summary_tail.read_available()
            association_rows = association_tail.read_available()
            events = timestamped_events(summary_rows, association_rows)
            if events:
                process_events(analyzer, events)
            else:
                time.sleep(args.poll_interval)
    finally:
        summary_tail.close()
        association_tail.close()


def self_test():
    with tempfile.TemporaryDirectory() as temporary_dir:
        empty_csv = Path(temporary_dir) / "delayed_header.csv"
        empty_csv.touch()

        def publish_header():
            time.sleep(0.05)
            empty_csv.write_text("timestamp,value\n", encoding="utf-8")

        publisher = threading.Thread(target=publish_header)
        publisher.start()
        follower = CsvFollower(
            empty_csv, from_start=False, wait_for_header=True
        )
        publisher.join(timeout=1.0)
        assert follower.fields == ["timestamp", "value"]
        follower.close()

    history = SortHistory(history_seconds=10.0)
    history.add(1, SortSample(10.0, 359.0, 4.0))
    history.add(1, SortSample(12.0, 1.0, 8.0))
    result = history.interpolate(1, 11.0, tolerance_s=1.1)
    assert result is not None
    assert abs(signed_circular_error_deg(result.relative_az, 0.0)) < 1.0e-9
    assert abs(result.elevation - 6.0) < 1.0e-9
    assert signed_circular_error_deg(2.0, 359.0) == 3.0
    states = parse_track_states({
        "timestamp": "20.0",
        "track_states": "3:12.5,6.25,0.1,-0.2;bad",
    })
    assert len(states) == 1
    assert states[0][0] == 3
    assert states[0][1].relative_az == 12.5

    class MemoryOutput:
        def __init__(self):
            self.rows = []

        def write(self, row):
            self.rows.append(row)

    output = MemoryOutput()
    args = SimpleNamespace(
        history_seconds=10.0,
        rolling_window=10,
        pairs=[],
        pair_mode="auto",
        sync_tolerance=0.6,
        print_interval=1.0e9,
    )
    analyzer = AlignmentAnalyzer(args, output)
    analyzer.consume_summary({
        "timestamp": "100.0",
        "ui_ids": "1",
        "track_states": "1:20.0,5.0,0.0,0.0;2:90.0,8.0,0.0,0.0",
    })
    analyzer.consume_summary({
        "timestamp": "101.0",
        "ui_ids": "1",
        "track_states": "1:22.0,7.0,0.0,0.0",
    })
    assert 2 not in analyzer.history.samples
    analyzer.consume_association({
        "timestamp": "101.0",
        "event": "RID_UI_FUSION_SUMMARY",
        "cycle": "9",
    })
    analyzer.consume_association({
        "timestamp": "101.0",
        "event": "SORT_TRACK",
        "cycle": "9",
        "sort_track_id": "1",
        "sort_map_az": "32.0",
        "sort_el": "7.0",
    })
    analyzer.consume_association({
        "timestamp": "101.0",
        "event": "RID_TRACK",
        "cycle": "9",
        "rid_ui_id": "4",
        "rid_id": "RID-TEST",
        "rid_map_az": "30.0",
        "rid_elevation_deg": "4.0",
        "rid_render_timestamp": "100.5",
        "device_heading_deg": "10.0",
    })
    analyzer.flush_cycle()
    assert len(output.rows) == 1
    assert float(output.rows[0]["az_error_unaligned_deg"]) == 2.0
    assert float(output.rows[0]["az_error_aligned_deg"]) == 1.0
    assert float(output.rows[0]["el_error_unaligned_deg"]) == 3.0
    assert float(output.rows[0]["el_error_aligned_deg"]) == 2.0

    with tempfile.TemporaryDirectory() as temporary_dir:
        state_path = Path(temporary_dir) / "learned.json"
        learner = OnlineMatchLearner(
            state_path=state_path,
            min_samples=4,
            confirm_updates=2,
            bootstrap_max_az_deg=30.0,
        )
        last_selected = []
        for sequence, az_error, el_error in (
            (1, 5.0, -2.0),
            (2, 5.2, -2.1),
            (3, 4.8, -1.9),
            (4, 5.1, -2.0),
            (5, 4.9, -2.2),
        ):
            last_selected = learner.process_cycle([{
                "sort_track_id": 1,
                "rid_id": "RID-LEARN",
                "rid_measurement_seq": sequence,
                "rid_render_timestamp": 100.0 + sequence,
                "cycle_timestamp": 100.0 + sequence,
                "board": "BOARD_1",
                "cam": 2,
                "logic_id": 3,
                "az_error_aligned_deg": az_error,
                "el_error_aligned_deg": el_error,
            }], now_ts=100.0 + sequence)
        learner.close()
        assert last_selected
        summary = learner.model_summaries()[0]
        assert summary["ready"]
        assert abs(summary["az"]["bias"] - 5.0) < 0.15
        assert abs(summary["el"]["bias"] + 2.0) < 0.15
        saved = json.loads(state_path.read_text(encoding="utf-8"))
        recommendation = next(iter(saved["models"].values()))["recommended"]
        assert recommendation["ready"]
        assert recommendation["az"]["gate"] <= 8.0

        multi = OnlineMatchLearner(
            state_path=state_path,
            min_samples=4,
            confirm_updates=2,
            ambiguity_margin=0.20,
        )
        for sequence in (20, 21):
            multi_candidates = [
                {
                    "sort_track_id": 10,
                    "rid_id": "RID-A",
                    "rid_measurement_seq": sequence,
                    "rid_render_timestamp": 300.0 + sequence,
                    "board": "BOARD_1",
                    "cam": 2,
                    "logic_id": 3,
                    "az_error_aligned_deg": 5.1,
                    "el_error_aligned_deg": -2.1,
                },
                {
                    "sort_track_id": 10,
                    "rid_id": "RID-B",
                    "rid_measurement_seq": sequence,
                    "rid_render_timestamp": 300.0 + sequence,
                    "board": "BOARD_1",
                    "cam": 2,
                    "logic_id": 3,
                    "az_error_aligned_deg": 7.0,
                    "el_error_aligned_deg": 0.0,
                },
                {
                    "sort_track_id": 20,
                    "rid_id": "RID-A",
                    "rid_measurement_seq": sequence,
                    "rid_render_timestamp": 300.0 + sequence,
                    "board": "BOARD_1",
                    "cam": 2,
                    "logic_id": 3,
                    "az_error_aligned_deg": 7.2,
                    "el_error_aligned_deg": 0.0,
                },
                {
                    "sort_track_id": 20,
                    "rid_id": "RID-B",
                    "rid_measurement_seq": sequence,
                    "rid_render_timestamp": 300.0 + sequence,
                    "board": "BOARD_1",
                    "cam": 2,
                    "logic_id": 3,
                    "az_error_aligned_deg": 4.9,
                    "el_error_aligned_deg": -1.9,
                },
            ]
            multi_selected = multi.process_cycle(
                multi_candidates, now_ts=300.0 + sequence
            )
        assert len(multi_selected) == 2
        assert all("confirmed_now" in item["learning_state"] for item in multi_selected)
        multi.close()

        ambiguous = OnlineMatchLearner(
            state_path=state_path,
            min_samples=4,
            confirm_updates=1,
            ambiguity_margin=0.20,
        )
        ambiguous_candidates = []
        for sort_id in (30, 40):
            for rid_id in ("RID-C", "RID-D"):
                ambiguous_candidates.append({
                    "sort_track_id": sort_id,
                    "rid_id": rid_id,
                    "rid_measurement_seq": 30,
                    "rid_render_timestamp": 330.0,
                    "board": "BOARD_1",
                    "cam": 2,
                    "logic_id": 3,
                    "az_error_aligned_deg": 5.0,
                    "el_error_aligned_deg": -2.0,
                })
        assert ambiguous.process_cycle(ambiguous_candidates, now_ts=330.0) == []
        ambiguous.close()

        false_alarm = OnlineMatchLearner(
            state_path=Path(temporary_dir) / "false_alarm.json",
            min_samples=4,
            confirm_updates=2,
            bootstrap_trajectory_samples=6,
            bootstrap_min_motion_deg=1.0,
            bootstrap_max_shape_p95_deg=2.5,
            bootstrap_max_trend_rmse_deg=1.5,
        )
        false_alarm_selected = []
        for sequence in range(1, 9):
            rid_az = 40.0 + 0.5 * sequence
            shared = {
                "rid_id": "RID-MOVING",
                "rid_measurement_seq": sequence,
                "rid_render_timestamp": 400.0 + sequence,
                "rid_map_az_deg": rid_az,
                "rid_el_deg": 5.0,
                "board": "BOARD_2",
                "cam": 1,
                "logic_id": 7,
            }
            false_alarm_selected = false_alarm.process_cycle(
                [
                    {
                        **shared,
                        "sort_track_id": 1,
                        "az_error_aligned_deg": 5.0 + 0.05 * (sequence % 2),
                        "el_error_aligned_deg": -1.0,
                    },
                    {
                        **shared,
                        "sort_track_id": 99,
                        "az_error_aligned_deg": 15.0 - 1.2 * sequence,
                        "el_error_aligned_deg": -1.0,
                    },
                ],
                now_ts=400.0 + sequence,
            )
        assert false_alarm_selected
        assert all(int(item["sort_track_id"]) == 1 for item in false_alarm_selected)
        assert 99 not in false_alarm.confirmed
        false_alarm.close()

        integrated_output = MemoryOutput()
        integrated_args = SimpleNamespace(
            history_seconds=20.0,
            rolling_window=10,
            pairs=[],
            pair_mode="learned",
            sync_tolerance=0.6,
            print_interval=math.inf,
            learner_state=Path(temporary_dir) / "integrated.json",
            learner_min_samples=4,
            learner_confirm_updates=2,
            learner_bootstrap_max_az=30.0,
            learner_ambiguity_margin=0.20,
            learner_hold_seconds=3.0,
            learner_bootstrap_trajectory_samples=4,
            learner_bootstrap_min_motion=1.0,
            learner_bootstrap_max_shape_p95=2.5,
            learner_bootstrap_max_trend_rmse=1.5,
        )
        integrated = AlignmentAnalyzer(integrated_args, integrated_output)
        for sequence in range(1, 7):
            timestamp = 200.0 + sequence
            integrated.consume_summary({
                "timestamp": str(timestamp),
                "ui_ids": "1",
                "track_states": "1:25.0,3.0,0.0,0.0",
            })
            integrated.consume_association({
                "timestamp": str(timestamp),
                "event": "RID_UI_FUSION_SUMMARY",
                "cycle": str(sequence),
            })
            integrated.consume_association({
                "timestamp": str(timestamp),
                "event": "SORT_TRACK",
                "cycle": str(sequence),
                "sort_track_id": "1",
                "board": "BOARD_1",
                "cam": "2",
                "logic_id": "3",
                "sort_map_az": "35.0",
                "sort_el": "3.0",
            })
            integrated.consume_association({
                "timestamp": str(timestamp),
                "event": "RID_TRACK",
                "cycle": str(sequence),
                "rid_id": "RID-INTEGRATED",
                "rid_ui_id": "1",
                "rid_measurement_seq": str(sequence),
                "rid_render_timestamp": str(timestamp),
                "rid_map_az": "30.0",
                "rid_elevation_deg": "5.0",
                "device_heading_deg": "10.0",
            })
        integrated.flush_cycle()
        assert integrated_output.rows
        assert integrated_output.rows[-1]["learning_state"] == "confirmed"
        assert int(integrated_output.rows[-1]["learned_ready"]) == 1
        integrated.learner.close()
    print("[Align] self-test passed")


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Align production SORT history to RID timestamps and measure "
            "azimuth/elevation residuals without opening hardware devices."
        )
    )
    parser.add_argument(
        "--logs-dir", default=str(DEFAULT_LOG_DIR),
        help="directory searched recursively for the latest matching log pair",
    )
    parser.add_argument(
        "--association-file",
        help="explicit rid_association_*.csv path",
    )
    parser.add_argument(
        "--summary-file",
        help="explicit track_summary_*.csv path; inferred from association file",
    )
    parser.add_argument(
        "--output",
        help="aligned output CSV; defaults beside the association log",
    )
    parser.add_argument(
        "--pair",
        dest="pairs",
        action="append",
        type=parse_pair,
        default=[],
        metavar="SORT_ID:RID_ID",
        help="known physical identity pair; repeat for multiple targets",
    )
    parser.add_argument(
        "--pair-mode",
        choices=("auto", "learned", "logged", "all"),
        default="auto",
        help=(
            "auto only pairs one SORT with one RID; learned performs persistent "
            "online calibration and conservative shadow matching; logged uses "
            "the current stateless pair; all emits every candidate"
        ),
    )
    parser.add_argument(
        "--learner-state", default=str(DEFAULT_LEARNER_STATE),
        help="persistent JSON model used by --pair-mode learned",
    )
    parser.add_argument(
        "--learner-min-samples", type=int, default=12,
        help="trusted samples required before multi-target learned matching",
    )
    parser.add_argument(
        "--learner-confirm-updates", type=int, default=3,
        help="distinct RID updates required to confirm a proposed identity",
    )
    parser.add_argument(
        "--learner-bootstrap-max-az", type=float, default=30.0,
        help="broad azimuth safety gate while a camera model is uncalibrated",
    )
    parser.add_argument(
        "--learner-ambiguity-margin", type=float, default=0.20,
        help="minimum normalized best-versus-second-best score separation",
    )
    parser.add_argument(
        "--learner-hold-seconds", type=float, default=3.0,
        help="identity reservation time through short SORT/RID gaps",
    )
    parser.add_argument(
        "--learner-bootstrap-trajectory-samples", type=int, default=12,
        help="distinct RID samples required to reject false SORT tracks by shape",
    )
    parser.add_argument(
        "--learner-bootstrap-min-motion", type=float, default=2.0,
        help="minimum RID azimuth or elevation span for false-alarm bootstrap",
    )
    parser.add_argument(
        "--learner-bootstrap-max-shape-p95", type=float, default=2.5,
        help="maximum centered residual p95 for trajectory-shape bootstrap",
    )
    parser.add_argument(
        "--learner-bootstrap-max-trend-rmse", type=float, default=1.5,
        help="maximum residual-increment RMSE for trajectory-shape bootstrap",
    )
    parser.add_argument(
        "--no-follow", action="store_true",
        help="process existing files once and exit",
    )
    parser.add_argument(
        "--from-start", action="store_true",
        help="include existing rows before following new rows",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="live duration in seconds; <=0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--sync-tolerance", type=float, default=0.75,
        help="maximum seconds from RID time to each interpolation endpoint",
    )
    parser.add_argument(
        "--history-seconds", type=float, default=60.0,
        help="SORT history retained for time alignment",
    )
    parser.add_argument(
        "--rolling-window", type=int, default=60,
        help="recent aligned samples used by live p95/bias output",
    )
    parser.add_argument(
        "--print-interval", type=float, default=1.0,
        help="minimum live console interval per selected pair",
    )
    parser.add_argument(
        "--poll-interval", type=float, default=0.10,
        help="file-tail poll interval in seconds",
    )
    parser.add_argument(
        "--wait-for-logs", type=float, default=30.0,
        help="seconds to wait for an auto-discovered log pair; <=0 waits forever",
    )
    parser.add_argument(
        "--log-not-before", type=float, default=0.0,
        help=(
            "only auto-select log files whose modification time is at or "
            "after this Unix timestamp"
        ),
    )
    parser.add_argument("--self-test", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.self_test:
        self_test()
        return 0

    summary_path, association_path = resolve_log_pair(args)
    output_path = (
        Path(args.output).resolve()
        if args.output
        else default_output_path(association_path)
    )
    print(f"[Align] SORT source: {summary_path}")
    print(f"[Align] RID source:  {association_path}")
    print(f"[Align] Output:      {output_path}")
    print("[Align] Residual convention: SORT - RID")

    output = OutputLog(output_path)
    analyzer = AlignmentAnalyzer(args, output)
    try:
        if args.no_follow:
            run_offline(args, summary_path, association_path, analyzer)
        else:
            run_follow(args, summary_path, association_path, analyzer)
    except KeyboardInterrupt:
        print("\n[Align] Stopped by user")
    finally:
        analyzer.print_final_summary()
        output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
