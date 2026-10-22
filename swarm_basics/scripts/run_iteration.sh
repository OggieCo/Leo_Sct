#!/usr/bin/env bash
# run_iteration.sh — collect ONE extra full pass over the matrix (14 scenarios
# x 3 arms) with the standard reduced-rate + CUDA configuration.
#
# Usage (INSIDE the container):
#   cd /root/ros2_ws/src/swarm_basics
#   nohup bash scripts/run_iteration.sh 6 >/dev/null 2>&1 &
#   tail -f scripts/batch_logs/iteration_6.log
#
# Properties:
#  - Runs are tagged SWARM_BATCH=batch_<N>, so each cell gets
#    <MODE>/<scenario>/batch_<N>_run_<ts>/  (rename to batch_<N> afterwards).
#  - Resume-safe: a cell is SKIPPED if a completed batch_<N> run folder
#    (summary.csv) already exists — just re-run the script to retry failures.
#  - Between cells: waits for the sim to tear down (stop_sim.sh as fallback)
#    and sleeps COOLDOWN seconds (default 0 = no cooldown).
#  - Ctrl+C stops the current cell gracefully (start_experiment SIGINT
#    cleanup) and exits; rerun later to continue.
set -u

ITER="${1:-6}"
BATCH="batch_${ITER}"
RUN_TIME="${RUN_TIME:-300}"     # DRIVING time per cell (from goal-node start)
COOLDOWN="${COOLDOWN:-0}"       # seconds between cells (0 = none; user request 2026-10-12)

# Background jobs of a non-interactive shell get SIGINT/SIGQUIT ignored, which
# would make the cell's auto-stop ('kill -INT $$') silently fail and the cell
# run forever (seen 2026-10-12).  The same reset run_matrix.sh uses:
RESET_PREFIX=()
if env --default-signal=SIGINT true 2>/dev/null; then
    RESET_PREFIX=(env --default-signal=SIGINT,SIGQUIT)
fi

cd "$(dirname "$0")/.."          # -> swarm_basics/
ROOT=/root/ros2_ws/src
SCENARIOS=(2_rovers_1_humans 2_rovers_2_humans 2_rovers_3_humans
           3_rovers_1_humans 3_rovers_2_humans 3_rovers_3_humans
           3_rovers_and_obstacles
           4_rovers_1_humans 4_rovers_2_humans 4_rovers_3_humans
           5_rovers_1_humans 5_rovers_2_humans 5_rovers_3_humans
           5_rovers_and_obstacles)
ARMS=(no reactive ai)
declare -A MODE=( [no]=Nav2 [reactive]=BT [ai]=BT_LLM )

LOGDIR="$ROOT/swarm_basics/scripts/batch_logs"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/iteration_${ITER}.log"
log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*" | tee -a "$LOG"; }

cell_done() {  # scenario arm -> 0 if a completed batch_<N> run exists
    local sc="$1" arm="$2" base="$ROOT/results/coverage_logs/${MODE[$arm]}/$sc"
    # accept the normalised plain name (batch_7) as well as the name a fresh
    # run gets (batch_7_run_<timestamp>) — the goal is "this cell already has a
    # finished run in this iteration", whatever the folder is called.
    [ -f "$base/${BATCH}/summary.csv" ] && return 0
    compgen -G "$base/${BATCH}_run_*/summary.csv" >/dev/null 2>&1
}

current_child=""
on_int() {
    if [[ -n "$current_child" ]]; then
        log "interrupt — stopping current cell gracefully"
        kill -INT "$current_child" 2>/dev/null || true
        wait "$current_child" 2>/dev/null || true
    fi
    log "stopped by user — resume anytime: bash scripts/run_iteration.sh $ITER"
    exit 0
}
trap on_int INT TERM

total=$(( ${#SCENARIOS[@]} * ${#ARMS[@]} ))
n=0
start_all=$(date +%s)
failed=()

log "iteration $ITER — $total cells ($BATCH, lidar 5Hz / cam 3Hz 320x240, YOLO cuda, run-time ${RUN_TIME}s)"

for sc in "${SCENARIOS[@]}"; do
    for arm in "${ARMS[@]}"; do
        n=$((n+1))
        if cell_done "$sc" "$arm"; then
            log "[$n/$total] skip ${arm}|${sc} — ${BATCH} already complete"
            continue
        fi
        log "[$n/$total] START ${arm}|${sc} (mode=${MODE[$arm]}, ${BATCH})"
        t0=$(date +%s)
        # Signal reset (see RESET_PREFIX above): lets the cell's auto-stop and
        # our INT forwarding actually work on this background child.
        "${RESET_PREFIX[@]}" \
        SWARM_BATCH="$BATCH" \
        SWARM_LIDAR_HZ=5 SWARM_CAMERA_HZ=3 SWARM_CAMERA_W=320 SWARM_CAMERA_H=240 \
        YOLO_DEVICE=cuda \
            ./scripts/start_experiment.sh --scenario "$sc" --bt "$arm" \
                --goals fixed --headless --run-time "$RUN_TIME" &
        current_child=$!
        wait "$current_child"; rc=$?
        current_child=""
        dt=$(( $(date +%s) - t0 ))
        if [[ $rc -eq 0 ]]; then
            log "[$n/$total] DONE  ${arm}|${sc}  (${dt}s)"
        else
            log "[$n/$total] FAIL  ${arm}|${sc}  rc=$rc (${dt}s) — retried on next invocation"
            failed+=("${arm}|${sc}")
        fi

        # wait for teardown (up to 30 s), then stop_sim.sh as a last resort
        for _i in $(seq 1 30); do
            pgrep -f 'ign gazebo' >/dev/null 2>&1 || break
            sleep 1
        done
        if pgrep -f 'ign gazebo' >/dev/null 2>&1; then
            log "cleanup: sim still alive — running stop_sim.sh"
            bash scripts/stop_sim.sh >/dev/null 2>&1 || true
        fi

        if [[ $n -lt $total ]]; then
            log "cooldown ${COOLDOWN}s"
            sleep "$COOLDOWN"
        fi
    done
done

mins=$(( ($(date +%s) - start_all) / 60 ))
log "iteration $ITER finished — $((total - ${#failed[@]})) ok / ${#failed[@]} failed, ${mins} min"
for f in "${failed[@]:-}"; do
    [[ -n "$f" ]] && log "  retry needed: $f"
done
