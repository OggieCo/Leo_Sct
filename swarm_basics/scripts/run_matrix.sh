#!/usr/bin/env bash
# =============================================================================
# run_matrix.sh — unattended headless batch for the thesis experiment matrix
#
# Runs the arms x scenarios matrix SEQUENTIALLY (never in parallel: a second
# stack would collide on the same world/topics and the laptop is already at
# ~11/14 GB RAM).  Each cell is one headless run of start_experiment.sh,
# which ends itself via its --run-time auto-stop (and flushes all CSVs).
#
# Properties:
#   * round-robin order (scenario -> repeats -> arms) so thermal/slowdown
#     drift over hours does not bias a single arm
#   * RESUME: finished cells are recorded in $STATE_DIR/completed.tsv and
#     skipped on the next invocation (safe to re-run after a crash/reboot)
#   * a cell is only marked done if a run_ folder with summary.csv (and
#     goal_stats.csv unless GOALS=walk) actually appeared in the expected
#     results/coverage_logs/<MODE>/<scenario>/ directory AND every rover
#     really drove (path length >= PATH_MIN_M, robot count matches the
#     scenario) — frozen-robot runs are detected instead of passing
#   * failed cells are QUARANTINED: their run_ folder is moved to
#     results/coverage_logs/_failed_runs/ and the cell is retried on the
#     next invocation (nothing is deleted)
#   * stale FastDDS shared-memory segments are cleared between cells
#     (they accumulate after forced shutdowns and caused DDS flakiness)
#   * every run is bounded by `timeout` (graceful SIGINT -> cleanup), and
#     leftover processes are cleaned up between cells
#   * Ctrl+C / `pkill -INT -f run_matrix.sh` stops the CURRENT cell
#     gracefully (the run's own shutdown flushes every CSV) and exits;
#     re-run the script to resume — the interrupted cell is simply redone
#     (its partial run_ folder stays in results/ and should be ignored when
#     aggregating: duration_sec well below RUN_TIME)
#   * everything is appended to $STATE_DIR/batch.log
#
# Usage (INSIDE the container):
#   bash /root/ros2_ws/src/swarm_basics/scripts/run_matrix.sh       # resume & run
#   DRY_RUN=1 bash /root/ros2_ws/src/swarm_basics/scripts/run_matrix.sh
#   nohup bash /root/ros2_ws/src/swarm_basics/scripts/run_matrix.sh \
#         >/dev/null 2>&1 &                                        # detached
#   tail -f /root/ros2_ws/src/swarm_basics/scripts/batch_logs/matrix/batch.log
#
# Fresh start: delete the state dir (rm -rf scripts/batch_logs/matrix).
# =============================================================================
set -u

# ------------------------------ configuration -------------------------------
ARMS=(no reactive ai)             # no = Nav2 baseline, reactive = BT, ai = BT+LLM
SCENARIOS=(1 2 3 4 5 6 7 8 9 10)  # 1:1r1h  2:1r2h  3:1r3h  4:2r1h  5:2r2h
                                  # 6:2r3h  7:3r1h  8:3r2h  9:3r3h  10:3r+obst
GOALS="fixed"                     # fixed = same tours (most comparable) | random | walk
RUN_TIME=300                      # seconds per run — KEEP IDENTICAL for every cell
REPEATS=1                         # >1 = repeat each cell (mean/std for the thesis)
COOLDOWN=20                       # seconds to cool down between runs
READY_TIMEOUT=240                 # max wait for Nav2 readiness (start_experiment.sh)
PATH_MIN_M=0.5                    # integrity: every rover must drive >= this many metres
QUARANTINE_DIR="/root/ros2_ws/src/results/coverage_logs/_failed_runs"
DRY_RUN="${DRY_RUN:-0}"           # 1 = print the cell list, execute nothing
# -----------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP="$SCRIPT_DIR/start_experiment.sh"
GEN="$SCRIPT_DIR/../worlds/scenario_generator.py"
BASE_DIR="/root/ros2_ws/src/results/coverage_logs"
STATE_DIR="$SCRIPT_DIR/batch_logs/matrix"
COMPLETED="$STATE_DIR/completed.tsv"

[[ -f "$EXP" ]] || { echo "ERROR: $EXP not found" >&2; exit 1; }
mkdir -p "$STATE_DIR"
touch "$COMPLETED"

# Non-interactive bash sets SIGINT/SIGQUIT to IGNORE for `&` background jobs,
# which would make the interrupt handler unable to stop the current cell —
# same trick as start_experiment.sh: reset the signals via env.
HAVE_ENV_RESET=""
env --default-signal=SIGINT true 2>/dev/null && HAVE_ENV_RESET="env-reset"
RESET_PREFIX=()
[[ -n "$HAVE_ENV_RESET" ]] && RESET_PREFIX=(env --default-signal=SIGINT,SIGQUIT)

CURRENT_CHILD=""   # PID of the running cell (set while a cell is executing)

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$STATE_DIR/batch.log"; }

mode_of() {  # arm -> results MODE subfolder (same rule as the launch files)
    case "$1" in
        no)       echo Nav2 ;;
        reactive) echo BT ;;
        ai)       echo BT_LLM ;;
        *)        echo "?" ;;
    esac
}

scenario_name() {  # scenario number -> name (empty if invalid)
    python3 "$GEN" --scenario "$1" --print-name 2>/dev/null
}

cell_key() { printf '%s|%s|%s' "$1" "$2" "$3"; }

cell_done() { grep -qxF "$1" "$COMPLETED" 2>/dev/null; }

leftovers_exist() {
    pgrep -f 'ros2_ws/install|ros_gz_bridge|slam_toolbox|nav2_' >/dev/null 2>&1
}

cleanup_leftovers() {
    log "cleanup: signalling leftover processes (same list start_experiment.sh suggests)"
    pkill -INT -f 'ros2_ws/install' 2>/dev/null || true
    pkill -INT -f 'ros_gz_bridge'   2>/dev/null || true
    pkill -INT -f 'slam_toolbox'    2>/dev/null || true
    pkill -INT -f 'nav2_'           2>/dev/null || true
    sleep 5
}

clean_shm() {  # drop stale FastDDS segments (nothing runs between cells)
    if ls /dev/shm/fastrtps_* >/dev/null 2>&1; then
        rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
        log "cleanup: cleared stale FastDDS /dev/shm segments"
    fi
}

quarantine_run() {  # arm scenario start_epoch — move THIS cell's failed folder out
    local mode name dir newest dest
    mode="$(mode_of "$1")"; name="$(scenario_name "$2")"
    [[ -n "$name" ]] || return 0
    dir="$BASE_DIR/$mode/$name"
    newest="$(ls -1dt "$dir"/run_* 2>/dev/null | head -1)"
    [[ -n "$newest" ]] || return 0
    [[ "$(stat -c %Y "$newest" 2>/dev/null || echo 0)" -ge "$(( $3 - 120 ))" ]] || return 0
    mkdir -p "$QUARANTINE_DIR"
    dest="$QUARANTINE_DIR/${mode}_${name}_$(basename "$newest")"
    if mv "$newest" "$dest" 2>/dev/null; then
        log "quarantine: moved failed run -> $dest"
    fi
}

verify_run() {  # arm scenario start_epoch -> 0 = results verified & complete
    local mode name dir newest
    mode="$(mode_of "$1")"
    name="$(scenario_name "$2")"
    [[ -n "$name" ]] || return 1
    dir="$BASE_DIR/$mode/$name"
    [[ -d "$dir" ]] || return 1
    newest="$(ls -1dt "$dir"/run_* 2>/dev/null | head -1)"
    [[ -n "$newest" ]] || return 1
    # folder must belong to THIS cell (created when the run started)
    [[ "$(stat -c %Y "$newest" 2>/dev/null || echo 0)" -ge "$(( $3 - 120 ))" ]] || return 1
    [[ -f "$newest/summary.csv" ]] || return 1
    if [[ "$GOALS" != "walk" ]]; then
        [[ -f "$newest/goal_stats.csv" ]] || return 1
    fi
    # INTEGRITY: every rover must actually have driven (a frozen robot still
    # produced full CSVs and used to be marked DONE — 2026-09-13), and the
    # robot count must match the scenario name (1_/2_/3_rovers_...).
    # NB: coverage_plotter writes summary.csv with CRLF line endings (python
    # csv default) — compare NUMERICALLY so a trailing \r cannot false-fail
    # the check (2026-09-13: it quarantined a perfectly good run).
    awk -F, -v min="$PATH_MIN_M" -v want="${name%%_*}" '
        $1 == "total_robots"   { have = $2 + 0 }
        $1 ~ /_path_length_m$/ { if ($2 + 0 < min) bad = 1 }
        END { exit (bad || have != (want + 0)) }
    ' "$newest/summary.csv" || return 1
    return 0
}

run_cell() {  # arm scenario repeat
    local arm="$1" scen="$2" rep="$3" start rc timeout_s
    local key; key="$(cell_key "$arm" "$scen" "$rep")"
    if [[ "$DRY_RUN" == "1" ]]; then
        log "DRY-RUN would run: $key (mode=$(mode_of "$arm"), name=$(scenario_name "$scen"))"
        return 0
    fi
    if leftovers_exist; then
        cleanup_leftovers
        if leftovers_exist; then
            log "ABORT $key: leftover processes still running — retry next invocation"
            return 1
        fi
    fi
    clean_shm
    start=$(date +%s)
    timeout_s=$(( READY_TIMEOUT + RUN_TIME + 180 ))
    log "START $key  (mode=$(mode_of "$arm"), timeout ${timeout_s}s)"
    # Run in the background so on_interrupt() knows the PID and can stop it
    # gracefully through timeout -> start_experiment.sh's SIGINT cleanup.
    "${RESET_PREFIX[@]}" timeout --signal=INT --kill-after=90 "$timeout_s" \
        bash "$EXP" --bt "$arm" --scenario "$scen" --goals "$GOALS" \
                    --run-time "$RUN_TIME" --headless \
        >"$STATE_DIR/run_${arm}_${scen}_${rep}.out" 2>&1 &
    CURRENT_CHILD=$!
    wait "$CURRENT_CHILD"
    rc=$?
    CURRENT_CHILD=""
    sleep 3
    # NOTE: done-ness is judged by the RESULTS, not rc — even a timeout-killed
    # run flushes its CSVs through the script's graceful SIGINT cleanup.
    if verify_run "$arm" "$scen" "$start"; then
        printf '%s\n' "$key" >> "$COMPLETED"
        log "DONE  $key  (rc=$rc, results verified)"
        return 0
    fi
    log "FAIL  $key  (rc=$rc, results NOT verified — not marked done)"
    quarantine_run "$arm" "$scen" "$start"
    cleanup_leftovers
    return 1
}

# ------------------------- interrupt handling ---------------------------------
on_interrupt() {
    trap '' INT TERM            # ignore a second Ctrl+C while cleaning up
    log "interrupted — stopping the current run gracefully, then exiting"
    log "resume anytime: bash $0  (state: $STATE_DIR)"
    if [[ -n "$CURRENT_CHILD" ]] && kill -0 "$CURRENT_CHILD" 2>/dev/null; then
        kill -INT "$CURRENT_CHILD" 2>/dev/null || true  # timeout -> run's cleanup
        local i
        for i in {1..30}; do        # let the run flush its CSVs (max 30 s)
            kill -0 "$CURRENT_CHILD" 2>/dev/null || break
            sleep 1
        done
        kill -TERM "$CURRENT_CHILD" 2>/dev/null || true
    fi
    exit 130
}
# HUP too: closing the terminal window (foreground batch) takes the same
# graceful path instead of leaving an orphaned simulation behind.
trap on_interrupt INT TERM HUP

# ------------------------------- main loop -----------------------------------

total=$(( ${#ARMS[@]} * ${#SCENARIOS[@]} * REPEATS ))
log "Matrix: ${#ARMS[@]} arms x ${#SCENARIOS[@]} scenarios x $REPEATS repeat(s) = $total runs"
log "Estimated duration: ~$(( total * (RUN_TIME + 120 + COOLDOWN) / 60 )) min (plus retries)"
log "State dir: $STATE_DIR  (completed so far: $(wc -l < "$COMPLETED" | tr -d ' '))"

i=0
for scen in "${SCENARIOS[@]}"; do
    for ((rep=1; rep<=REPEATS; rep++)); do
        for arm in "${ARMS[@]}"; do
            i=$((i+1))
            key="$(cell_key "$arm" "$scen" "$rep")"
            if cell_done "$key"; then
                log "[$i/$total] SKIP  $key (already completed)"
                continue
            fi
            log "[$i/$total] $key"
            run_cell "$arm" "$scen" "$rep" || true
            if [[ "$DRY_RUN" != "1" ]]; then
                sleep "$COOLDOWN"
            fi
        done
    done
done

log "Batch finished — $(wc -l < "$COMPLETED" | tr -d ' ')/$total cells completed. Logs: $STATE_DIR"
