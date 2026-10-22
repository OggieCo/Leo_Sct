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
#   * PASSES: an invocation first finishes any pending cells; if every cell
#     is already completed, it starts a NEW PASS automatically — completed.tsv
#     is archived (completed_<ts>.tsv) and all cells run again, so new
#     run_<ts> folders accumulate in the same MODE/scenario dirs (multiple
#     runs of one cell = the repeats for the thesis analysis).
#     ITERATIONS=N chains N passes in one invocation (e.g. overnight runs).
#   * single-instance lock: refuses to start if another run_matrix.sh is up.
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
ARMS=(${SWARM_ARMS:-no reactive ai})   # no = Nav2 baseline, reactive = BT, ai = BT+LLM
                                  # env override: SWARM_ARMS="reactive ai" bash run_matrix.sh
# SCALED MATRIX (2026-09-17, supervisor feedback): rovers 2..5 x humans 1..3.
#   1:2r1h   2:2r2h   3:2r3h   4:3r1h   5:3r2h   6:3r3h
#   7:4r1h   8:4r2h   9:4r3h  10:5r1h  11:5r2h  12:5r3h
#  13:3r+obst  14:5r+obst
# !! The scenario NUMBERING CHANGED vs the old 1-3 rover matrix (1 now = 2r1h):
#    state from batch_1/batch_2 (batch_logs/matrix) must NOT be reused —
#    archive it and start a fresh pass when switching to this matrix.
# Scope down if time is short — e.g. crowded columns only:
#   SCENARIOS=(3 6 9 12 14)
SCENARIOS=(${SWARM_SCENARIOS:-1 2 3 4 5 6 7 8 9 10 11 12 13 14})
                                  # env override: SWARM_SCENARIOS="11 12" bash run_matrix.sh
GOALS="fixed"                     # fixed = same tours (most comparable) | random | walk
RUN_TIME=300                      # seconds per run — KEEP IDENTICAL for every cell
REPEATS=1                         # >1 = repetitions of every cell.  The repeat
                                  # index is passed as SWARM_RUN_SEED, so each
                                  # repetition replays the same random sequence
                                  # (= "10 seeds" for the thesis statistics)
COOLDOWN=20                       # seconds to cool down between runs
READY_TIMEOUT=240                 # max wait for Nav2 readiness; start_experiment.sh
                                  # auto-extends to 300 s for 4-5 rover scenarios
PATH_MIN_M=0.5                    # integrity: every rover must drive >= this many metres
QUARANTINE_DIR="/root/ros2_ws/src/results/coverage_logs/_failed_runs"
ITERATIONS="${ITERATIONS:-1}"     # full passes this invocation (each pass re-runs all cells)
DRY_RUN="${DRY_RUN:-0}"           # 1 = print the cell list, execute nothing
# -----------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP="$SCRIPT_DIR/start_experiment.sh"
GEN="$SCRIPT_DIR/../worlds/scenario_generator.py"
BASE_DIR="/root/ros2_ws/src/results/coverage_logs"
STATE_DIR="$SCRIPT_DIR/batch_logs/matrix"
COMPLETED="$STATE_DIR/completed.tsv"
BATCH_FILE="$STATE_DIR/.current_batch"   # campaign batch of the CURRENT pass
BATCH_NAME=""                            # e.g. batch_2 — see the pass loop

[[ -f "$EXP" ]] || { echo "ERROR: $EXP not found" >&2; exit 1; }
mkdir -p "$STATE_DIR"
touch "$COMPLETED"

latest_batch_num() {  # highest existing batch number anywhere under BASE_DIR
    local n
    n=$(ls -d "$BASE_DIR"/*/*/batch_* 2>/dev/null | sed -n 's#.*/batch_\([0-9][0-9]*\).*#\1#p' | sort -n | tail -1)
    echo "${n:-0}"
}

# Single-instance lock: a kernel advisory lock (flock) held for the whole run
# and auto-released on exit/crash — prevents two concurrent batches from
# fighting over the same world/ports.  (pgrep-based detection was dropped:
# it false-positived on the invoking wrapper's command line.)
exec 9>"$STATE_DIR/.lock"
if ! flock -n 9; then
    echo "ERROR: another run_matrix.sh is already running (lock: $STATE_DIR/.lock)." >&2
    echo "       Stop it first:  pkill -INT -f run_matrix.sh" >&2
    exit 1
fi

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
    pgrep -f 'ros2_ws/install|ros_gz_bridge|slam_toolbox|nav2_|leo_gz.launch.py|ign gazebo|robot_state_publisher' >/dev/null 2>&1
}

cleanup_leftovers() {
    log "cleanup: signalling leftover processes (same list start_experiment.sh suggests)"
    # Patterns now also cover the launch wrappers and the sim itself: on
    # 2026-09-14 a wedged run left 'ros2 launch leo_gz.launch.py' + 'ign
    # gazebo' + robot_state_publishers alive; signalling only the node
    # patterns never cleared them and every later cell aborted on leftovers.
    # Escalate INT -> TERM -> KILL until the process list is actually clean.
    local pats=('ros2_ws/install' 'ros_gz_bridge' 'slam_toolbox' 'nav2_'
                'leo_gz.launch.py' 'ign gazebo' 'robot_state_publisher')
    local wave p
    for wave in "INT:5" "TERM:3" "KILL:2"; do
        for p in "${pats[@]}"; do pkill -"${wave%%:*}" -f "$p" 2>/dev/null || true; done
        sleep "${wave##*:}"
        leftovers_exist || break
    done
}

clean_shm() {  # drop stale FastDDS segments + semaphores (nothing runs between cells)
    if ls /dev/shm/*fastrtps* /dev/shm/*fastdds* >/dev/null 2>&1; then
        rm -f /dev/shm/*fastrtps* /dev/shm/*fastdds* 2>/dev/null || true
        log "cleanup: cleared stale FastDDS /dev/shm segments"
    fi
}

quarantine_run() {  # arm scenario start_epoch — move THIS cell's failed folder out
    local mode name dir newest dest
    mode="$(mode_of "$1")"; name="$(scenario_name "$2")"
    [[ -n "$name" ]] || return 0
    dir="$BASE_DIR/$mode/$name"
    newest="$(ls -1dt "$dir/${BATCH_NAME:-batch_1}_run_"* 2>/dev/null | head -1)"
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
    newest="$(ls -1dt "$dir/${BATCH_NAME:-batch_1}_run_"* 2>/dev/null | head -1)"
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
    # Margin covers the full recovery chain: 300 s first gate + two surgical
    # restarts (120 s re-gates) + one full-stack restart (180 s re-gate).
    timeout_s=$(( READY_TIMEOUT + 600 + RUN_TIME + 300 ))
    # Seeds: with REPEATS>1 the repeat index IS the run seed — the SAME seed
    # is used for every arm of this repeat (fair A/B, reproducible replay).
    local -a seed_env=()
    (( REPEATS > 1 )) && seed_env=(SWARM_RUN_SEED="$rep")
    log "START $key  (mode=$(mode_of "$arm"), batch=${BATCH_NAME:-batch_1}, seed=${rep}, timeout ${timeout_s}s)"
    # Run in the background so on_interrupt() knows the PID and can stop it
    # gracefully through timeout -> start_experiment.sh's SIGINT cleanup.
    # 9>&-: do NOT leak the flock fd into the cell's process tree — a child
    # surviving a hard kill (2026-09-14: wedged sim) would otherwise keep the
    # run lock held forever and every later invocation would refuse to start.
    "${RESET_PREFIX[@]}" "${seed_env[@]}" timeout --signal=INT --kill-after=90 "$timeout_s" \
        bash "$EXP" --bt "$arm" --scenario "$scen" --goals "$GOALS" \
                    --run-time "$RUN_TIME" --headless \
        >"$STATE_DIR/run_${arm}_${scen}_${rep}.out" 2>&1 9>&- &
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
# Persist the cell total for auto_retry_when_pending.sh (it must not hardcode
# 30 anymore — the scaled 2026-09-17 matrix has 42 cells, x10 seeds = 420).
printf '%s\n' "$total" > "$STATE_DIR/.total_cells"
log "Matrix: ${#ARMS[@]} arms x ${#SCENARIOS[@]} scenarios x $REPEATS repeat(s) = $total cells"
log "Estimated duration per pass: ~$(( total * (RUN_TIME + 120 + COOLDOWN) / 60 )) min (plus retries)"

BATCH_NAME=""
[[ -f "$BATCH_FILE" ]] && BATCH_NAME="$(cat "$BATCH_FILE")"

for ((pass=1; pass<=ITERATIONS; pass++)); do
    # A fully completed pass rolls over into a NEW pass: archive the
    # completed-list so every cell runs again -> new run_<ts> folders appear
    # in the same MODE/scenario dirs (the dataset accumulates passes).
    done_now=$(wc -l < "$COMPLETED" | tr -d ' ')
    if (( done_now >= total )) && [[ "$DRY_RUN" != "1" ]]; then
        arch="$STATE_DIR/completed_$(date +%Y%m%d_%H%M%S).tsv"
        mv "$COMPLETED" "$arch"
        touch "$COMPLETED"
        log "pass complete ($done_now/$total) — archived '$(basename "$arch")', starting a NEW pass"
        # NEW campaign -> NEW batch tag (strategy/seed/batch_X): every run
        # of this pass is named <MODE>/<scenario>/<batch>_run_<id>.
        BATCH_NAME="batch_$(( $(latest_batch_num) + 1 ))"
        printf '%s\n' "$BATCH_NAME" > "$BATCH_FILE"
    elif [[ -z "$BATCH_NAME" ]]; then
        # First launch / lost state: join the newest existing batch.
        BATCH_NAME="batch_$(latest_batch_num)"
        [[ "$BATCH_NAME" == "batch_0" ]] && BATCH_NAME="batch_1"
        if [[ "$DRY_RUN" != "1" ]]; then
            printf '%s\n' "$BATCH_NAME" > "$BATCH_FILE"
        fi
    fi
    export SWARM_BATCH="$BATCH_NAME"
    iter_no=$(( $(ls -1 "$STATE_DIR"/completed_*.tsv 2>/dev/null | wc -l) + 1 ))
    log "=== ITERATION $iter_no — pass $pass/$ITERATIONS (completed so far: $(wc -l < "$COMPLETED" | tr -d ' ')/$total; batch: $BATCH_NAME) ==="

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

    log "Pass $pass finished — $(wc -l < "$COMPLETED" | tr -d ' ')/$total cells completed."
done

log "All $ITERATIONS pass(es) done. State: $STATE_DIR"
