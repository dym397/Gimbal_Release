#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

# Do not start Python until all RID-version serial devices are present.  The
# Python process creates a timestamped run directory immediately, so letting
# systemd restart it while hardware is absent would generate one empty log
# directory every RestartSec interval.
GIMBAL_PORT="${GIMBAL_PORT:-/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.2:1.0-port0}"
GPS_PORT="${GPS_PORT:-/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.1:1.0-port0}"
RID_PORT="${RID_PORT:-/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.3:1.0-port0}"
HARDWARE_WAIT_INTERVAL="${HARDWARE_WAIT_INTERVAL:-2}"
HARDWARE_WAIT_LOG_INTERVAL="${HARDWARE_WAIT_LOG_INTERVAL:-30}"

last_missing=""
last_log_ts=0
while true; do
    missing=()
    for device in \
        "gimbal:${GIMBAL_PORT}" \
        "gps:${GPS_PORT}" \
        "rid:${RID_PORT}"; do
        role="${device%%:*}"
        path="${device#*:}"
        if [[ -z "$path" || ! -c "$path" || ! -r "$path" || ! -w "$path" ]]; then
            missing+=("${role}=${path:-unset}")
        fi
    done

    if (( ${#missing[@]} == 0 )); then
        echo "[HardwareWait] ready: gimbal=${GIMBAL_PORT}, gps=${GPS_PORT}, rid=${RID_PORT}"
        break
    fi

    missing_text="${missing[*]}"
    now_ts="$(date +%s)"
    if [[ "$missing_text" != "$last_missing" ]] \
        || (( now_ts - last_log_ts >= HARDWARE_WAIT_LOG_INTERVAL )); then
        echo "[HardwareWait] waiting; missing/unusable: ${missing_text}"
        last_missing="$missing_text"
        last_log_ts="$now_ts"
    fi
    sleep "$HARDWARE_WAIT_INTERVAL"
done

export GIMBAL_PORT GPS_PORT RID_PORT

# Run the angle-alignment analyzer as a read-only sidecar of the main process.
# It follows only log files created for this service invocation, so an old run
# can never be selected during the short startup race before FieldLogger opens
# the new files. All output remains attached to this systemd service/journal.
RID_SORT_ALIGNMENT_ENABLE="${RID_SORT_ALIGNMENT_ENABLE:-0}"
RID_SORT_ALIGNMENT_PRINT_INTERVAL="${RID_SORT_ALIGNMENT_PRINT_INTERVAL:-10}"
RID_SORT_ALIGNMENT_SYNC_TOLERANCE="${RID_SORT_ALIGNMENT_SYNC_TOLERANCE:-0.75}"
RID_SORT_ALIGNMENT_PAIR_MODE="${RID_SORT_ALIGNMENT_PAIR_MODE:-learned}"
RID_SORT_ALIGNMENT_PAIRS="${RID_SORT_ALIGNMENT_PAIRS:-}"
RID_SORT_ALIGNMENT_WAIT_FOR_LOGS="${RID_SORT_ALIGNMENT_WAIT_FOR_LOGS:-0}"
RID_SORT_ALIGNMENT_LOG_DIR="${RID_SORT_ALIGNMENT_LOG_DIR:-${LOG_DIR:-logs}}"
RID_SORT_ALIGNMENT_MODEL_PATH="${RID_SORT_ALIGNMENT_MODEL_PATH:-$SCRIPT_DIR/calibration/rid_sort_alignment_model.json}"

if [[ "$RID_SORT_ALIGNMENT_ENABLE" != "1" ]]; then
    exec python3 "$SCRIPT_DIR/main_tracking_v9.py"
fi

# Keep sub-second precision. During a rapid systemd restart the previous main
# process can flush its old log files in the same wall-clock second; a whole-
# second cutoff would then let the analyzer attach to that previous run.
service_started_ts="$(date +%s.%N)"
main_pid=""
alignment_pid=""

cleanup_children() {
    if [[ -n "$alignment_pid" ]] && kill -0 "$alignment_pid" 2>/dev/null; then
        kill "$alignment_pid" 2>/dev/null || true
    fi
    if [[ -n "$main_pid" ]] && kill -0 "$main_pid" 2>/dev/null; then
        kill "$main_pid" 2>/dev/null || true
    fi
    if [[ -n "$alignment_pid" ]]; then
        wait "$alignment_pid" 2>/dev/null || true
    fi
    if [[ -n "$main_pid" ]]; then
        wait "$main_pid" 2>/dev/null || true
    fi
}

trap cleanup_children EXIT
trap 'exit 143' TERM INT

python3 "$SCRIPT_DIR/main_tracking_v9.py" &
main_pid="$!"

alignment_args=(
    "$SCRIPT_DIR/tools_py/rid_sort_alignment_live.py"
    --logs-dir "$RID_SORT_ALIGNMENT_LOG_DIR"
    --log-not-before "$service_started_ts"
    --wait-for-logs "$RID_SORT_ALIGNMENT_WAIT_FOR_LOGS"
    --print-interval "$RID_SORT_ALIGNMENT_PRINT_INTERVAL"
    --sync-tolerance "$RID_SORT_ALIGNMENT_SYNC_TOLERANCE"
    --pair-mode "$RID_SORT_ALIGNMENT_PAIR_MODE"
    --learner-state "$RID_SORT_ALIGNMENT_MODEL_PATH"
)
if [[ -n "$RID_SORT_ALIGNMENT_PAIRS" ]]; then
    IFS=',' read -r -a configured_pairs <<< "$RID_SORT_ALIGNMENT_PAIRS"
    for configured_pair in "${configured_pairs[@]}"; do
        configured_pair="${configured_pair//[[:space:]]/}"
        if [[ -n "$configured_pair" ]]; then
            alignment_args+=(--pair "$configured_pair")
        fi
    done
fi

echo "[AlignService] starting: print_interval=${RID_SORT_ALIGNMENT_PRINT_INTERVAL}s, sync_tolerance=${RID_SORT_ALIGNMENT_SYNC_TOLERANCE}s, pair_mode=${RID_SORT_ALIGNMENT_PAIR_MODE}, pairs=${RID_SORT_ALIGNMENT_PAIRS:-auto}, model=${RID_SORT_ALIGNMENT_MODEL_PATH}, logs=${RID_SORT_ALIGNMENT_LOG_DIR}"
python3 "${alignment_args[@]}" &
alignment_pid="$!"

set +e
wait "$main_pid"
main_status="$?"
set -e
main_pid=""

# The analyzer is diagnostic-only. Its failure must not terminate or restart
# the main tracking process; the service exit status always follows main.
exit "$main_status"
