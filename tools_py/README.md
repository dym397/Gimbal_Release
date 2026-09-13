# tools_py

Utility, calibration, RKNN conversion, display-test, and UDP receiver scripts.
These are not required by the normal gimbal tracking runtime.

## Special RID identity replay

`replay_special_rid_identity.py` replays the two whitelisted RID aircraft and
their SORT generations through the same production family-link logic. For the
`20260912_032537` test, start at the XDB old-UI28/SORT177 checkpoint:

```bash
python3 tools_py/replay_special_rid_identity.py \
  --log-dir logs/20260912_032537 \
  --start-ts 1789156230.355754
```

The expected result is XDB/UI1 with family `{177}` and ST22Q/UI2 with family
`{189,196,202,205,238}`; SORT238 remains owned by ST22Q. Add `--json` for a
machine-readable summary. The tool is read-only and never opens serial ports,
sends UDP packets, or controls the gimbal.

## Live SORT/RID angle alignment

`rid_sort_alignment_live.py` is a read-only diagnostic for measuring the
azimuth and elevation difference between production SORT tracks and RID
tracks. It tails the matching `track_summary_*.csv` and
`rid_association_*.csv` files; it does not open the RID serial port, bind the
detection UDP port, send UI packets, or control the gimbal.

For the cleanest calibration, run one physical target at a time. Start
`main_tracking_v9.py` with field logging enabled, then run:

```powershell
python tools_py/rid_sort_alignment_live.py --logs-dir logs
```

The tool automatically selects the newest run containing both input files.
It interpolates the high-rate SORT history to `rid_render_timestamp`, so the
default RID UI delay (currently about `0.8s`) is removed before comparison.
The console and output CSV report both unaligned and aligned errors. Error
signs are always `SORT - RID`.

For a known target in a multi-target test, specify the physical identity pair:

```powershell
python tools_py/rid_sort_alignment_live.py `
  --association-file logs\RUN\rid_association_TIMESTAMP.csv `
  --pair 3:RID_TARGET_ID
```

Repeat `--pair` for multiple known pairs. Without explicit pairs, `auto` mode
only analyzes cycles containing exactly one SORT and one RID target. This
prevents the diagnostic from deciding identity using the same angular error it
is intended to measure. `--pair-mode logged` can be used to inspect the
current stateless UI pairing, but those results are provisional rather than
ground truth.

Offline analysis of an existing run is also supported:

```powershell
python tools_py/rid_sort_alignment_live.py `
  --association-file logs\RUN\rid_association_TIMESTAMP.csv `
  --no-follow --from-start
```

Useful output columns include `az_error_unaligned_deg`,
`az_error_aligned_deg`, `el_error_aligned_deg`, `rid_render_delay_s`, and
`sort_alignment_gap_s`. For higher-rate calibration, reduce
`RID_ASSOC_LOG_INTERVAL` during the test; this changes diagnostic log cadence
only and does not change the tracker or gimbal-control behavior.

The systemd startup path enables this analyzer by default. The service wrapper
starts `main_tracking_v9.py` first and then starts the analyzer as a read-only
child process. Both console streams are visible through the same journal:

```bash
journalctl -u gimbal-tracking -f
```

The analyzer follows only files created after the current service invocation,
so it cannot attach to an old run directory during startup. Main-process exit
stops the analyzer; analyzer failure does not stop or restart the main process.
The defaults can be overridden in `/etc/default/gimbal-tracking`:

```bash
# Set to 0 to run only main_tracking_v9.py.
RID_SORT_ALIGNMENT_ENABLE=1

# Rolling statistics are printed once per selected pair at this interval.
RID_SORT_ALIGNMENT_PRINT_INTERVAL=10

# Disable the unrelated once-per-second [Live] runtime status line.
PRINT_LIVE_STATUS=0

# Maximum timestamp gap accepted on either side of SORT interpolation.
RID_SORT_ALIGNMENT_SYNC_TOLERANCE=0.75

# learned bootstraps from a one-SORT/one-RID flight, persists per-camera
# calibration, and later proposes conservative multi-target bindings.
RID_SORT_ALIGNMENT_PAIR_MODE=learned

# Persistent samples plus derived bias/noise/gate/weight recommendations.
RID_SORT_ALIGNMENT_MODEL_PATH=/home/linaro/gimbal/calibration/rid_sort_alignment_model.json

# Optional known multi-target identities, comma-separated.
# RID_SORT_ALIGNMENT_PAIRS=3:RID_A,4:RID_B
```

After changing the installed unit file, reload and restart it with:

```bash
sudo systemctl daemon-reload
sudo systemctl restart gimbal-tracking
```

### Five-minute single-UAV learning flight

The service now defaults to `RID_SORT_ALIGNMENT_PAIR_MODE=learned`. The
analyzer only consumes SORT states that have passed `UI_TRACK_CONFIRM_HITS`
and are present in `ui_tracks`; internal tracks that have only passed the
earlier SORT confirmation are excluded. The analyzer still keys history by
the internal SORT `track_id`. The numeric external UI ID is merely allocated
when sending and is not an additional filter stage.

A flight with one valid RID identity may contain several UI-confirmed SORT
tracks because persistent detector false alarms can also pass the UI hit gate.
The learner therefore has two bootstrap paths:

- one SORT plus one RID: direct conservative confirmation;
- multiple SORT plus one RID: compare synchronized trajectory shapes over at
  least 12 distinct RID updates.

For every candidate, the shape bootstrap removes its median angular bias and
measures centered residual p95 plus residual-increment RMSE. The real visual
track should move with RID, leaving an approximately constant residual; a
static or unrelated false track normally produces a changing residual as the
UAV moves. The best candidate must also be sufficiently separated from the
second-best candidate before it can be confirmed and used for learning.

Shape bootstrap requires at least `2deg` of RID azimuth or elevation movement.
If the UAV only hovers while two candidates are similarly stationary, their
identity is physically unobservable from angle trajectories alone and the
learner intentionally returns no match. A useful five-minute calibration
flight should therefore include horizontal motion and an altitude change,
rather than five minutes of hovering.

After confirmation, the learner stores per-camera calibration in
`calibration/rid_sort_alignment_model.json`.

The learned matcher uses robust, interpretable statistics rather than a neural
network:

1. SORT is interpolated to each RID render timestamp.
2. For every `board/cam/logic_id`, azimuth and elevation residuals are stored
   as `SORT - RID`.
3. Residual median estimates systematic bias; `1.4826 * MAD` estimates noise.
4. The learned gate is based on `max(3.5*sigma, 1.25*p95)` with bounded safety
   limits.
5. Azimuth/elevation weights are inverse-variance weights; elevation is capped
   at 40% because it depends on the station and RID altitude datums.
6. Multi-target candidates use the bias-corrected normalized cost:

```text
cost = w_az * abs(az_error - az_bias) / az_gate
     + w_el * abs(el_error - el_bias) / el_gate
```

7. A rectangular Hungarian assignment enforces one-to-one matching. Small
   best-versus-second-best margins are rejected as ambiguous, three distinct
   RID updates are required for confirmation, and confirmed identity is held
   briefly through gaps.

Only confirmed or unambiguous single-target samples update the model. Proposed
bindings remain shadow diagnostics and do not alter UI packets, SORT, master
selection, gimbal commands, or strike safety.

During the flight, `[AlignLearn]` reports contain:

- `az_bias` / `el_bias`: learned systematic angular offset;
- `az_sigma` / `el_sigma`: robust residual noise;
- `az_gate` / `el_gate`: recommended acceptance threshold;
- `weights=(az,el)`: automatically learned matching weights;
- `samples` and `ready`: distinct trusted RID samples and whether the model is
  ready for multi-target shadow matching.

The JSON model also stores the recent trusted samples and a `recommended`
snapshot, including centered p95 errors. A single-UAV flight calibrates angular
agreement but does not by itself validate target crossing or close-bearing
multi-UAV identity; those require a subsequent multi-target shadow test.

There is no fixed five-minute runtime limit. The service analyzer runs until
the main process/service stops, so a real test can transition directly from a
single-target calibration phase into a two-target shadow-matching phase. The
single-target confirmed binding is retained, while the second target must pass
the same one-to-one, ambiguity, and repeated-update checks.

Long-running memory is bounded: each camera keeps at most 500 recent trusted
azimuth/elevation residuals, SORT interpolation retains 60 seconds, each
bootstrap candidate retains a bounded trajectory window, inactive false-alarm
candidate histories expire after 120 seconds, and the learned-update dedupe
cache is capped. CSV production logs continue growing for the duration of one
service run, so disk space should still be monitored for tests lasting many
hours.
