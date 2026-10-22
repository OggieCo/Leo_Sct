#!/usr/bin/env bash
# finish_campaign.sh — top every cell of the matrix up to TARGET runs (default
# 10) by re-running the cells that are short, each with its next free
# batch_<N> tag.
#
# Usage (INSIDE the container):
#   cd /root/ros2_ws/src/swarm_basics
#   nohup bash scripts/finish_campaign.sh >/dev/null 2>&1 &
#   tail -f scripts/batch_logs/finish_campaign.log
#
# Properties:
#  - Resume-safe: each cell's run count is re-read from disk before every run,
#    so the script can be killed and relaunched at any time; cells already at
#    TARGET are skipped.
#  - A run counts only if its folder is a plain batch_<N> directory holding
#    summary.csv — timestamped strays from killed runs are ignored.
#  - Uses the standard reduced-rate + CUDA configuration, same as
#    run_iteration.sh, and inherits the scenario env of start_experiment.sh
#    (SWARM_LIDAR_HZ 5, camera 3 Hz 320x240, SWARM_SCAN_THRESHOLD 0.10).
#  - Env overrides: TARGET=10 RUN_TIME=300 COOLDOWN=0.
set -u

TARGET="${TARGET:-10}"
RUN_TIME="${RUN_TIME:-300}"
COOLDOWN="${COOLDOWN:-0}"
ROOT=/root/ros2_ws/src
SWARM_DIR="$ROOT/swarm_basics"

SCENARIOS=(2_rovers_1_humans 2_rovers_2_humans 2_rovers_3_humans
           3_rovers_1_humans 3_rovers_2_humans 3_rovers_3_humans
           3_rovers_and_obstacles
           4_rovers_1_humans 4_rovers_2_humans 4_rovers_3_humans
           5_rovers_1_humans 5_rovers_2_humans 5_rovers_3_humans
           5_rovers_and_obstacles)
ARMS=(no reactive ai)
declare -A MODE=( [no]=Nav2 [reactive]=BT [ai]=BT_LLM )

# Background jobs of a non-interactive shell ignore SIGINT, which would make the
# cell's auto-stop ('kill -INT $$') fail silently in start_experiment.sh.
RESET_PREFIX=()
if env --default-signal=SIGINT true 2>/dev/null; then
    RESET_PREFIX=(env --default-signal=SIGINT,SIGQUIT)
fi

cd "$SWARM_DIR"
LOGDIR="$SWARM_DIR/scripts/batch_logs"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/finish_campaign.log"
log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*" | tee -a "$LOG"; }

count_runs() {  # dir -> number of valid runs (plain batch_<N> + summary.csv)
    local d="$1" f n=0
    for f in "$d"/batch_*; do
        [[ -d "$f" && -f "$f/summary.csv" ]] || continue
        [[ "$(basename "$f")" =~ ^batch_[0-9]+$ ]] || continue
        n=$((n+1))
    done
    echo "$n"
}

next_tag() {  # dir -> first free batch_<N>
    local d="$1" i=1
    while [[ -e "$d/batch_$i" ]]; do i=$((i+1)); done
    echo "batch_$i"
}

current_child=""
on_int() {
    if [[ -n "$current_child" ]]; then
        log "interrupt — stopping current cell gracefully"
        kill -INT "$current_child" 2>/dev/null || true
        wait "$current_child" 2>/dev/null || true
    fi
    log "stopped by user — relaunch any time to resume"
    exit 0
}
trap on_int INT TERM

missing=0
for sc in "${SCENARIOS[@]}"; do
    for arm in "${ARMS[@]}"; do
        have=$(count_runs "$ROOT/results/coverage_logs/${MODE[$arm]}/$sc")
        (( have < TARGET )) && missing=$((missing + TARGET - have))
    done
done
log "target ${TARGET} runs/cell — ${missing} run(s) missing"

n=0
for sc in "${SCENARIOS[@]}"; do
    for arm in "${ARMS[@]}"; do
        d="$ROOT/results/coverage_logs/${MODE[$arm]}/$sc"
        while :; do
            have=$(count_runs "$d")
            (( have >= TARGET )) && break
            tag=$(next_tag "$d")
            n=$((n+1))
            log "[$n/$missing] START ${arm}|${sc} ($tag, ${have}/${TARGET} done)"
            t0=$(date +%s)
            "${RESET_PREFIX[@]}" \
                SWARM_BATCH="$tag" \
                SWARM_LIDAR_HZ=5 SWARM_CAMERA_HZ=3 \
                SWARM_CAMERA_W=320 SWARM_CAMERA_H=240 \
                YOLO_DEVICE=cuda \
                ./scripts/start_experiment.sh --scenario "$sc" --bt "$arm" \
                    --goals fixed --headless --run-time "$RUN_TIME" &
            current_child=$!
            wait "$current_child"; rc=$?
            current_child=""
            dt=$(( $(date +%s) - t0 ))
            if [[ $rc -eq 0 ]]; then
                log "[$n] DONE  ${arm}|${sc} ($tag, ${dt}s)"
            else
                log "[$n] FAIL  ${arm}|${sc} ($tag, rc=$rc, ${dt}s) — skipped, relaunch retries"
                break
            fi

            # wait for teardown (up to 30 s), then stop_sim.sh as last resort
            for _i in $(seq 1 30); do
                pgrep -f 'ign gazebo' >/dev/null 2>&1 || break
                sleep 1
            done
            if pgrep -f 'ign gazebo' >/dev/null 2>&1; then
                log "cleanup: sim still alive — running stop_sim.sh"
                bash scripts/stop_sim.sh >/dev/null 2>&1 || true
            fi
            sleep "$COOLDOWN"
        done
    done
done
log "all cells at ${TARGET} runs — campaign complete"
