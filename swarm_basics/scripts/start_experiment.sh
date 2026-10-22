#!/usr/bin/env bash
# =============================================================================
# start_experiment.sh — one-command startup for the Leo Rover swarm simulation
#
# Replaces the manual multi-terminal procedure:
#   1. terminal 1: ros2 launch swarm_basics leo_gz.launch.py ...   -> this script
#   2. terminal 2: ros2 launch swarm_basics nav2_slam_all.launch.py -> this script
#   3. click "Play" in Gazebo GUI                                   -> done via
#                                                                      ign service
#   4. wait a couple of seconds                                     -> replaced by
#                                                                      a Nav2
#                                                                      readiness
#                                                                      gate
#   5. terminal 3: send goals (random_goals_all / fixed_goals_all)  -> --goals
#
# Usage (run inside the container, from anywhere):
#   scripts/start_experiment.sh --bt stock               # pure Nav2 baseline
#   scripts/start_experiment.sh --bt reactive            # reactive BT
#   scripts/start_experiment.sh --bt ai                  # BT + LLM layer
#   scripts/start_experiment.sh --bt ai --goals random   # + auto-start goals
#   scripts/start_experiment.sh --bt stock --headless    # no GUI (batch runs)
#
# Options:
#   --bt stock|reactive|ai  Behavior layer (required; 'no' = legacy alias)
#   --llm true|false        Start llm_planner (default: true only for --bt ai)
#   --detector yolo|depth   Human detector backend (default: yolo)
#   --headless              Run Gazebo without GUI (auto-runs, no Play button)
#   --goal / --goals ...    none (default) | random | fixed | walk
#   --world PATH            Override the simulation world (sim_world:=PATH)
#   --ready-timeout SEC     Max wait for Nav2 to become active (default 240;
#                           auto-extended to 300 when the scenario has 4+ robots)
#   --run-time SEC          Graceful auto-stop N seconds after READY
#                           (default 300 = 5 min; 0 disables the timer)
#   --dynamic-plots BOOL    Live coverage-map redraws during the run
#                           (default false; coverage map rendered at shutdown)
#   --raw-contacts BOOL     Log EVERY raw contact message to contacts_raw.csv
#                           (default false; events.csv keeps one
#                           CONTACT_BEGIN/CONTACT_END pair per episode)
#   --scenario NAME|N       Scenario to run: e.g. 3_rovers_3_humans,
#                           3_rovers_and_obstacles (numbers 1..10 also work);
#                           run folders are grouped under <MODE>/<name>/
#
# Press Ctrl+C to shut everything down (also saves all logs).
# =============================================================================
set -u

# ---------------------------------------------------------------------------
# Results/logs are written INSIDE this container (running as root) but they live
# on the HOST bind mount.  With the default umask 022 every run_* folder came
# out root-owned and 755, so the host user needed root just to delete it.
# umask 000 makes new dirs 777 / files 666, so the host user can manage results
# directly.  (Single-user dev container on a personal machine.)
# ---------------------------------------------------------------------------
umask 000

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --------------------------- duplicate-launch guard --------------------------
# Two start_experiment.sh instances on the same scenario fight over the same
# Gazebo/DDS resources (observed 2026-10-10: control loops missed their rates,
# action servers timed out, goals aborted).  An exclusive flock makes a second
# instance fail fast instead of corrupting the running one.
# (Do NOT replace this with a pgrep-based check — a pgrep guard was tried first
# and false-positived on its own command-substitution subshell, which inherits
# the script's cmdline.)
exec 9>/tmp/start_experiment.lock
if ! flock -n 9; then
    printf 'ERROR: another start_experiment.sh is already running — aborting.\n' >&2
    exit 1
fi

# ------------------------------- defaults -----------------------------------
BT=""
LLM=""
DETECTOR="yolo"
HEADLESS="false"
WORLD=""
GOALS="none"
READY_TIMEOUT=240
STALL_DETECT_SECS=90   # probe early-exit: no-progress robot while others ready
RUN_TIME=300
DYNAMIC_PLOTS="false"
RAW_CONTACTS="false"
SCENARIO=""

# ------------------------------- helpers ------------------------------------
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,42p' "$0" | cut -c3-
    exit "${1:-0}"
}

# Shutdown correctness depends on two things:
#  1. setsid — every launch gets its OWN process group, so we can signal the
#     whole tree (ros2 launch children are otherwise orphaned and keep running).
#  2. env --default-signal — bash sets SIGINT/SIGQUIT to IGNORE for background
#     jobs in a non-interactive script, so without this the launched processes
#     would ignore the graceful SIGINT we send them (verified: plain background
#     process survives kill -INT; with env reset it dies as expected).
HAVE_SETSID=""
command -v setsid >/dev/null 2>&1 && HAVE_SETSID="setsid"
HAVE_ENV_RESET=""
env --default-signal=SIGINT true 2>/dev/null && HAVE_ENV_RESET="env-reset"

start_bg() {  # start_bg <cmd...> — record the new process-group leader in PIDS
    if [[ -n "$HAVE_SETSID" && -n "$HAVE_ENV_RESET" ]]; then
        setsid env --default-signal=SIGINT,SIGQUIT "$@" &
    elif [[ -n "$HAVE_SETSID" ]]; then
        setsid "$@" &
    else
        "$@" &
    fi
    PIDS+=("$!")
}

signal_groups() {  # signal_groups <SIG> — signal every launch process group
    local sig="$1" pid
    for pid in "${PIDS[@]}"; do
        kill -"$sig" -- "-$pid" 2>/dev/null || kill -"$sig" "$pid" 2>/dev/null || true
    done
}

groups_alive() {
    local pid
    for pid in "${PIDS[@]}"; do
        kill -0 -- "-$pid" 2>/dev/null && return 0
        kill -0 "$pid" 2>/dev/null && return 0
    done
    return 1
}

# ------------------- runtime safety defaults (2026-09-23) -------------------
# These four settings live here so that EVERY entry point (run_matrix.sh or a
# manual ./start_experiment.sh) applies them — skipping them walks straight
# into the failure modes they prevent.  The environment can still override
# each one.
#
#  1) FD LIMIT — the container's soft limit is 1024, while a 5-rover stack opens
#     hundreds of DDS sockets and shared-memory segments.  Hitting EMFILE makes
#     participants silently lose discovery: their topics 'never appear', no
#     odom/map arrives and the readiness gate can NEVER pass (measured
#     2026-09-23).  Hard limit is 524288, so raising the soft limit is safe.
#  2) UDP-ONLY DDS TRANSPORT — shared memory costs one FD + one /dev/shm('port')
#     segment per participant-segment pair (301 files measured per boot), which
#     is the fastest route to fault 1.  UDPv4 took /dev/shm from 301 files to 0
#     with zero transport errors in the logs.
#  3) CLOCK RATE — Gazebo publishes /clock once per physics step (383 Hz
#     measured at 1 ms steps) and every sim-time node subscribes.  Tens of
#     thousands of callbacks/s froze costmaps ('time 0.7 s') and made SLAM drop
#     scans; the relay re-publishes the newest sample at this rate instead.
#  4) YOLO GATE — with YOLO_WAIT_GATE=1 the human detectors load torch + CUDA + the
#     model only after this script passes the Nav2 gate, so the 5x model-load
#     burst can never land inside bring-up or a recovery window.
ulimit -n 65536 2>/dev/null || true
[[ "$(ulimit -n)" -ge 4096 ]] || log "WARNING: fd limit is only $(ulimit -n) — DDS discovery may fail; raise it (ulimit -n 65536) or restart the container with --ulimit nofile=65536:524288"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export SWARM_CLOCK_HZ="${SWARM_CLOCK_HZ:-50}"
export YOLO_WAIT_GATE="${YOLO_WAIT_GATE:-1}"
# SLAM map fraction a rover must have mapped before its far tour goals are
# unlocked (fixed_goals.py reads this once at node start).  0.30 up to
# 2026-10-14, then 0.20, now 0.10: under 5-rover load SLAM grows the map too
# slowly to ever clear a 20% gate, so every rover was stuck on forward-scan
# goals (run_2026-10-14_17-40-49 / _20-01-14).
export SWARM_SCAN_THRESHOLD="${SWARM_SCAN_THRESHOLD:-0.10}"

log "Safety: nofile=$(ulimit -n) dds=$FASTDDS_BUILTIN_TRANSPORTS clock=${SWARM_CLOCK_HZ}Hz lidar=${SWARM_LIDAR_HZ:-5}Hz camera=${SWARM_CAMERA_HZ:-3}Hz yolo_gate=$YOLO_WAIT_GATE scan_threshold=$SWARM_SCAN_THRESHOLD"
# A thermally throttled host is the difference between a boot that passes and one
# that silently loses a robot, so record the active cap in the run log.
if [[ -r /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq ]]; then
    log "CPU cap: $(awk '{printf "%.1f", $1/1e6}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq) GHz (thermal-safe; see scripts/host_cpu_cap.sh)"
fi

# ------------------------------- arguments ----------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --bt)            BT="${2:-}"; shift 2 ;;
        --llm)           LLM="${2:-}"; shift 2 ;;
        --detector)      DETECTOR="${2:-}"; shift 2 ;;
        --headless)      HEADLESS="true"; shift ;;
        --gui)           HEADLESS="false"; shift ;;
        --goals|--goal)  GOALS="${2:-}"; shift 2 ;;
        --world)         WORLD="${2:-}"; shift 2 ;;
        --ready-timeout) READY_TIMEOUT="${2:-}"; shift 2 ;;
        --run-time)      RUN_TIME="${2:-}"; shift 2 ;;
        --dynamic-plots) DYNAMIC_PLOTS="${2:-}"; shift 2 ;;
        --raw-contacts)  RAW_CONTACTS="${2:-}"; shift 2 ;;
        --scenario)      SCENARIO="${2:-}"; shift 2 ;;
        --seed)          SCENARIO="${2:-}"; shift 2 ;;  # legacy alias
        -h|--help)       usage 0 ;;
        *)               die "Unknown argument: $1 (try --help)" ;;
    esac
done

[[ -n "$BT" ]] || { echo "Missing required --bt argument." >&2; usage 1; }
case "$BT" in
    stock|no)  BT="stock" ;;   # 'no' = legacy alias for the stock Nav2 tree
    reactive|ai) ;;
    *) die "--bt must be one of: stock, reactive, ai" ;;
esac
case "$GOALS" in
    none|random|fixed|walk) ;;
    *) die "--goals must be one of: none, random, fixed, walk" ;;
esac
[[ "$RUN_TIME" =~ ^[0-9]+$ ]] || die "--run-time must be a non-negative integer (seconds)"
case "$DYNAMIC_PLOTS" in
    true|false) ;;
    *) die "--dynamic-plots must be true or false" ;;
esac
case "$RAW_CONTACTS" in
    true|false) ;;
    *) die "--raw-contacts must be true or false" ;;
esac
if [[ -n "$SCENARIO" ]]; then
    [[ "$SCENARIO" =~ ^[0-9A-Za-z_-]+$ ]] || die "--scenario must be a number (1..10) or a scenario name (e.g. 2_rovers_1_humans)"
fi

# LLM default: only the 'ai' tree consumes it.
if [[ -z "$LLM" ]]; then
    if [[ "$BT" == "ai" ]]; then LLM="true"; else LLM="false"; fi
fi

# Run-mode (results/coverage_logs/<MODE>/) for EVERY node of the launch: same
# rule as spawn_multi_robots.launch.py.  The launch files export it for their
# own nodes, but the goal drivers are started directly by this script —
# without this export they defaulted to BT_LLM and misfiled goal_stats.csv
# (observed: BT run 2026-09-13_17-13-39 wrote into BT_LLM/3_rovers_3_humans).
if [[ "$BT" == "stock" ]]; then
    export SWARM_RUN_MODE="Nav2"
elif [[ "$LLM" == "true" ]]; then
    export SWARM_RUN_MODE="BT_LLM"
else
    export SWARM_RUN_MODE="BT"
fi

# ------------------------------- environment --------------------------------
# colcon's setup.bash references unset variables (e.g. COLCON_TRACE) without
# defaults, which is fatal under `set -u` — so relax nounset while sourcing.
set +u
if ! command -v ros2 >/dev/null 2>&1 && [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
if ! ros2 pkg prefix swarm_basics >/dev/null 2>&1 && [[ -f /root/ros2_ws/install/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /root/ros2_ws/install/setup.bash
fi
set -u
command -v ros2 >/dev/null 2>&1 || die "ros2 not found — source your ROS 2 environment first."

# ------------------------------- scenario -----------------------------------
# --scenario NAME|N: use worlds/scenarios/<name> — generated(if missing) world
# file + scenario.json that robot_config reads for the rover spawn poses; run
# folders go to results/coverage_logs/<MODE>/<name>/ (SWARM_SCENARIO).
# The name IS the scenario (e.g. 2_rovers_1_humans / 3_rovers_and_obstacles);
# the numbers 1..10 also resolve (--seed works as a legacy alias).
if [[ -n "$SCENARIO" ]]; then
    GEN="$SCRIPT_DIR/../worlds/scenario_generator.py"
    SCENARIO_NAME="$(python3 "$GEN" --scenario "$SCENARIO" --print-name 2>/dev/null || true)"
    [[ -n "$SCENARIO_NAME" ]] || die "Unknown scenario '$SCENARIO' — see SCENARIOS in worlds/scenario_generator.py"
    SCENARIO_DIR="$SCRIPT_DIR/../worlds/scenarios/$SCENARIO_NAME"
    if [[ ! -f "$SCENARIO_DIR/scenario.sdf" ]]; then
        log "Generating scenario $SCENARIO_NAME..."
        python3 "$GEN" --scenario "$SCENARIO" || die "Scenario generation failed ($SCENARIO)"
    fi
    export SWARM_SCENARIO="$SCENARIO_NAME"
    export SWARM_SCENARIO_JSON="$SCENARIO_DIR/scenario.json"
    [[ -z "$WORLD" ]] && WORLD="$SCENARIO_DIR/scenario.sdf"
fi

# World name (bridges/contact topics are built from this constant).
WORLD_NAME="$(python3 -c 'from swarm_basics.robot_config import WORLD_NAME; print(WORLD_NAME)' 2>/dev/null || true)"
WORLD_NAME="${WORLD_NAME:-human_world}"

# Robots to wait for (from the shared config — add/remove robots there).
mapfile -t ROBOTS < <(python3 - <<'PY' 2>/dev/null || true
from swarm_basics.robot_config import ROBOT_POSITIONS
for ns, *_ in ROBOT_POSITIONS:
    print(ns)
PY
)
if (( ${#ROBOTS[@]} == 0 )); then
    ROBOTS=(robot_0 robot_1)
fi

# 4-5 rover stacks boot heavier (a full SLAM/Nav2 stack per robot plus more
# DDS traffic): give the FIRST readiness window extra headroom so a single
# slow starter does not burn into the recovery loop (2026-09-17: 4-5 rover
# boots flaked right at the 240 s edge).  A larger explicit --ready-timeout wins.
if (( ${#ROBOTS[@]} >= 4 )) && (( READY_TIMEOUT <= 240 )); then
    READY_TIMEOUT=300
    log "4+ robots: extending Nav2 readiness timeout to ${READY_TIMEOUT}s"
fi

# 5 rovers: the LAST stack only starts at 20 + 12*4 = 68 s (nav2_slam_all.launch.py)
# and needs ~40 s to activate, so a HEALTHY boot already spends ~110 s of the 300 s
# window.  One surgical recovery then needs another 120 s re-gate — measured
# 2026-09-23, the 300 s window expired mid-recovery and escalated to a FULL restart
# (which is both the most expensive rung and the one that has produced TF-blind
# robots in the past).  Give the first window enough room to finish the common
# surgical path before escalating.
if (( ${#ROBOTS[@]} >= 5 )) && (( READY_TIMEOUT < 420 )); then
    READY_TIMEOUT=420
    log "5 robots: extending Nav2 readiness timeout to ${READY_TIMEOUT}s "
    log "(last Nav2 stack starts at 68 s; leaves room for one surgical recovery)"
fi

# The stall detector counts "no progress" from t=0 for EVERY robot, but the
# Nav2 bring-up is STAGGERED (nav2_slam_all.launch.py: 20 s + 12 s per robot,
# so robot_4's stack only starts at 68 s and activates ~30-40 s later).  A flat
# 90 s window therefore declared a robot stalled before it had even been
# launched — with 5 rovers it fired at 90 s and sent a HEALTHY boot straight
# into the recovery ladder.  Widen it past the last robot's start + activation.
if (( ${#ROBOTS[@]} >= 4 )); then
    STALL_DETECT_SECS=180
    log "4+ robots: extending stall detection to ${STALL_DETECT_SECS}s "
    log "(Nav2 bring-up is staggered 20 s + 12 s per robot)"
fi

# Ignition CLI (Fortress ships 'ign'; newer Gazebo releases use 'gz').
IGN=""
if command -v ign >/dev/null 2>&1; then IGN="ign"
elif command -v gz >/dev/null 2>&1; then IGN="gz"
fi

# ------------------------------- logs ---------------------------------------
# One run id for THIS launch, shared by every node through SWARM_RUN_ID: all
# CSVs (goal_stats.csv included) land in the same results/coverage_logs/<MODE>/
# [<scenario>/]run_<id> folder — even for nodes started only after the Nav2
# readiness gate ~30 s in, which used to splinter a second run_ folder
# (run_..._16-53-28).
RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)"
export SWARM_RUN_ID="$RUN_ID"
LOG_DIR="$SCRIPT_DIR/startup_logs/$RUN_ID"
mkdir -p "$LOG_DIR"

# ------------------------------- cleanup ------------------------------------
PIDS=()
NAV2_PIDS=()   # nav2/SLAM launch groups (targeted by surgical/full restarts)
WATCHDOG_PID=""
cleanup() {
    # Idempotent — may be reached via the INT/TERM handler AND the EXIT trap.
    [[ -n "${_CLEANED:-}" ]] && return 0
    _CLEANED=1
    trap - INT TERM
    [[ -n "$WATCHDOG_PID" ]] && kill "$WATCHDOG_PID" 2>/dev/null || true
    log "Shutting down (signalling every launch process group)..."

    signal_groups INT
    local i
    for i in {1..10}; do groups_alive || break; sleep 1; done
    if groups_alive; then
        log "Graceful shutdown timed out — sending SIGTERM..."
        signal_groups TERM
        for i in {1..5}; do groups_alive || break; sleep 1; done
    fi
    if groups_alive; then
        log "WARNING: processes still alive — sending SIGKILL."
        signal_groups KILL
        sleep 1
    fi

    wait 2>/dev/null || true
    if pgrep -f "ros2 launch swarm_basics" >/dev/null 2>&1; then
        log "WARNING: a leftover 'ros2 launch swarm_basics' process is still running."
    fi
    # Stale FastDDS shared-memory segments AND semaphores (sem.fastrtps_port*)
    # accumulate after forced (SIGKILL) shutdowns and caused SHM port-init
    # failures on later boots (2026-09-13; the sems slipped past the old
    # 'fastrtps_*' pattern until 2026-09-14).  Nothing of ours runs anymore
    # -> safe to clear them.
    if ! pgrep -f '/swarm_basics/lib/swarm_basics/' >/dev/null 2>&1; then
        # 2026-09-18: also clear '_port*' + '*_el' lock files — they slipped
        # past the old patterns and caused SHM port-init failures that stopped
        # spawners from ever receiving robot_description.
        rm -f /dev/shm/*fastrtps* /dev/shm/*fastdds* /dev/shm/_port* /dev/shm/*_el 2>/dev/null || true
    fi
    log "Stopped. Logs: $LOG_DIR"
}
trap 'cleanup; exit 0' INT TERM
# Cleanup on ANY exit too (die paths, unexpected errors) — wedged leftovers
# after a failed startup were a recurring source of blocked follow-up runs.
trap 'cleanup' EXIT

# ------------------------------- sanity check -------------------------------
if [[ -n "$IGN" ]] && $IGN service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/control"; then
    die "A Gazebo world '${WORLD_NAME}' is already running — stop it first."
fi
# A stale sim would corrupt the run's CSVs (shared topics, shared run folder),
# so refuse to start while leftover swarm_basics processes exist.
if pgrep -f 'ros2 launch swarm_basics' >/dev/null 2>&1 \
   || pgrep -f '/swarm_basics/lib/swarm_basics/' >/dev/null 2>&1; then
    die "Leftover swarm_basics processes detected from a previous run. Clean up first:
    pkill -INT -f 'ros2_ws/install'; pkill -INT -f 'ros_gz_bridge'; pkill -INT -f 'slam_toolbox'; pkill -INT -f 'nav2_'; sleep 3
    pgrep -af 'ros2_ws/install|ros_gz_bridge|slam_toolbox|nav2_' || echo all-clean"
fi

log "Mode: bt=$BT  llm=$LLM  detector=$DETECTOR  headless=$HEADLESS  goals=$GOALS  dynamic_plots=$DYNAMIC_PLOTS  raw_contacts=$RAW_CONTACTS  scenario=${SCENARIO:-none}"
[[ -n "$HAVE_SETSID" ]] || log "WARNING: 'setsid' not found — shutdown may not reach all child processes."
[[ -n "$HAVE_ENV_RESET" ]] || log "WARNING: 'env --default-signal' unavailable — graceful SIGINT may not reach the stack; shutdown will escalate to SIGTERM/SIGKILL."

# ------------------------------- 1) Gazebo ----------------------------------
SIM_ARGS=(headless:="$HEADLESS" detector:="$DETECTOR" enable_llm:="$LLM" bt:="$BT" dynamic_plots:="$DYNAMIC_PLOTS" log_raw_contacts:="$RAW_CONTACTS")
[[ -n "$WORLD" ]] && SIM_ARGS+=(sim_world:="$WORLD")

log "Starting Gazebo + robots: ros2 launch swarm_basics leo_gz.launch.py ${SIM_ARGS[*]}"
start_bg ros2 launch swarm_basics leo_gz.launch.py "${SIM_ARGS[@]}" >"$LOG_DIR/sim.log" 2>&1

# Programmatic "Play": GUI runs start paused; unpause once the world is up.
# (Headless mode already starts with '-r', so nothing to do there.)
if [[ "$HEADLESS" == "false" && -z "$IGN" ]]; then
    log "WARNING: no 'ign'/'gz' CLI found — cannot auto-unpause; click Play manually."
fi
if [[ "$HEADLESS" == "false" && -n "$IGN" ]]; then
    log "Waiting for the world to load, then unpausing (programmatic Play)..."
    deadline=$((SECONDS + 120))
    until $IGN service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/control"; do
        (( SECONDS < deadline )) || die "World service never appeared — check $LOG_DIR/sim.log"
        sleep 1
    done
    $IGN service -s "/world/${WORLD_NAME}/control" \
        --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean \
        --timeout 3000 --req 'pause: false' >/dev/null \
        && log "Simulation unpaused."
fi

# ------------------------------- 2) SLAM + Nav2 -----------------------------
# BRING-UP STRATEGY (2026-09-23) — SEQUENTIAL by default.
# Parallel bring-up (every stack launched at once, activation staggered 20+12i)
# still lost a robot regularly: with 4-5 stacks constructing simultaneously the
# laggard's lifecycle manager aborted its OWN activation ('bt_navigator/get_state
# service client: async_send_request failed'), and the 4-rover BT+LLM boot on
# 2026-09-23 needed two surgical restarts AND a full restart before the gate
# finally passed ~8 min after launch.
# Sequential bring-up gives every robot a clean window (one stack activates in
# ~20 s when it is alone on the machine) and repairs a bad robot BEFORE the next
# one starts, so a single flaky stack can no longer poison the whole boot.
# SWARM_BRINGUP=parallel restores the old behaviour.
BRINGUP="${SWARM_BRINGUP:-sequential}"
PER_ROBOT_TIMEOUT="${SWARM_PER_ROBOT_TIMEOUT:-90}"
SETTLE_AFTER_KILL="${SWARM_SETTLE_AFTER_KILL:-10}"

launch_nav2_subset() {  # one robot's Nav2/SLAM stack (own process group)
    start_bg ros2 launch swarm_basics nav2_slam_all.launch.py \
        bt:="$BT" robots:="$1" >>"$LOG_DIR/nav2.log" 2>&1
    NAV2_PIDS+=("${PIDS[${#PIDS[@]}-1]}")
}

launch_nav2_all() {  # every stack at once (parallel mode / full restart)
    start_bg ros2 launch swarm_basics nav2_slam_all.launch.py \
        bt:="$BT" >>"$LOG_DIR/nav2.log" 2>&1
    NAV2_PIDS+=("${PIDS[${#PIDS[@]}-1]}")
}

run_gate_single() {  # run_gate_single <ns> <timeout_s> — gate for ONE robot
    # --stall 0: the stall comparison needs "at least one robot ready", which
    # does not exist yet while robots are brought up one at a time.
    timeout --signal=INT --kill-after=10 "$(( $2 + 30 ))" \
        python3 "$SCRIPT_DIR/wait_nav2_ready.py" --stall 0 \
        --nav2-log "$LOG_DIR/nav2.log" --result-file "$GATE_RESULT" "$2" "$1"
}

# ------------------------------- 3) readiness gate --------------------------
# Robust in-process probe (scripts/wait_nav2_ready.py): ONE rclpy process,
# HARD checks per robot — bt_navigator lifecycle 'active' AND at least one
# message on {ns}/map (= SLAM is really running).  The old per-poll
# `ros2 lifecycle get` CLI was fragile under boot load (2026-09-13), and the
# 2026-09-14..17 flake (one slam_toolbox wedged at init, futex wait, zero
# output) is invisible to an "active" check alone.
#
# AUTO-RECOVERY v2 (2026-09-17, surgical): a wedged slam_toolbox (or an
# aborted lifecycle bringup) never heals, but a FRESH stack comes up cleanly.
# The probe exits early and names the failed robots (STALL / ABORTED /
# TIMEOUT + $GATE_RESULT).  Recovery ladder:
#   1) kill ONLY those robots' Nav2/SLAM processes and relaunch that subset
#      (robots:=robot_2[,robot_3]) — healthy robots keep running untouched.  x2
#   2) still failing -> FULL-stack restart (all nav2, SHM cleared)           x1
#   3) still failing -> fail fast; the batch retries the whole cell
NAV2_RETRY_TIMEOUT=180      # full-restart re-gate: ~3x a clean activation
SURGICAL_RETRY_TIMEOUT=120  # subset re-gate: one robot needs ~30-40 s
SURGICAL_RETRY_TIMEOUT_LONG=150   # 2nd surgical round: a slow SLAM gets more room
SURGICAL_RETRY_TIMEOUT_FINAL=180  # last light rung after the full restart
GATE_RESULT="$LOG_DIR/gate_result.txt"

robot_patterns() {  # robot_patterns <ns> — nav2-launch processes of ONE robot
    local ns="$1"
    printf '%s\n' \
        "${ns}_slam_toolbox" \
        "planner_server.*__ns:=/${ns}" \
        "controller_server.*__ns:=/${ns}" \
        "bt_navigator.*__ns:=/${ns}" \
        "behavior_server.*__ns:=/${ns}" \
        "lifecycle_manager.*__ns:=/${ns}"
}

robot_kill() {  # robot_kill <ns> — stop ONLY this robot's nav2 processes
    local ns="$1" p wave attempt
    for wave in "INT:3" "KILL:2"; do
        while IFS= read -r p; do
            pkill "-${wave%%:*}" -f "$p" 2>/dev/null || true
        done < <(robot_patterns "$ns")
        sleep "${wave##*:}"
    done
    # The old code reported "some processes survived SIGKILL" and then relaunched
    # ANYWAY — leaving two live stacks with the SAME node names in the same
    # namespace, whose duplicate services/parameter events break the lifecycle
    # bring-up.  That is a direct cause of surgical restarts failing to clear a
    # laggard (2026-09-23).  Verify, escalate on explicit PIDs, and report.
    for attempt in {1..5}; do
        local alive=() pid
        while IFS= read -r p; do
            while IFS= read -r pid; do
                [[ -n "$pid" ]] && alive+=("$pid")
            done < <(pgrep -f "$p" 2>/dev/null)
        done < <(robot_patterns "$ns")
        (( ${#alive[@]} == 0 )) && return 0
        log "  $ns: ${#alive[@]} process(es) still alive (pids: ${alive[*]}) — SIGKILL"
        kill -KILL "${alive[@]}" 2>/dev/null || true
        sleep 2
    done
    return 1
}

failed_robots() {  # robots flagged by the last probe run (space separated)
    sed -n 's/^FAILED //p' "$GATE_RESULT" 2>/dev/null
}

failed_noscan_robots() {  # robots whose LiDAR never reached the probe
    sed -n 's/^NOSCAN //p' "$GATE_RESULT" 2>/dev/null
}

ensure_scan_republishers() {  # ensure_scan_republishers <ns>...
    # The probe reports 'no_scan' when nothing ever arrives on
    # /{ns}/lidar/scan_clean.  That is a SIM-side fault: {ns}/lidar_republish
    # (started with the world, NOT with Nav2) rewrites Gazebo's mangled scan
    # frame, and without it SLAM has no input, so '{ns}/map' never appears.  A
    # Nav2/SLAM restart can never repair this — one reason a laggard could
    # survive every surgical round.
    # Only start it when it is really absent: a second publisher on the same
    # topic would duplicate every scan.
    local ns started=0
    for ns in $*; do
        pgrep -f "lidar_republish.*__ns:=/${ns} " >/dev/null 2>&1 && continue
        log "RECOVERY (sim-side): $ns lidar_republish is not running — restarting it"
        start_bg ros2 run swarm_basics lidar_republish --ros-args \
            -r "__ns:=/${ns}" -p use_sim_time:=true >>"$LOG_DIR/sim_republish.log" 2>&1
        started=1
    done
    (( started )) && sleep 5   # latch onto the Gazebo scan topic before the gate
    return 0
}

run_gate() {  # run_gate <timeout_s> — probe wrapped in a HARD external guard:
    # the probe itself wedged once in a DDS futex wait (2026-09-17 drill), so
    # SIGINT the python at t+30, SIGKILL 10 s later; a guard-kill simply counts
    # as "not ready" and the recovery ladder continues deterministically.
    local t="$1"
    timeout --signal=INT --kill-after=10 "$(( t + 30 ))" \
        python3 "$SCRIPT_DIR/wait_nav2_ready.py" --stall "$STALL_DETECT_SECS" \
        --nav2-log "$LOG_DIR/nav2.log" --result-file "$GATE_RESULT" \
        "$t" "${ROBOTS[@]}"
}

restart_subset() {  # restart_subset <ns> [<ns> ...] -> 0 if the gate passes
    local ns comma window="${SURGICAL_WINDOW:-$SURGICAL_RETRY_TIMEOUT}"
    _SURGICAL_BLOCKED=0
    comma="$(printf '%s' "$*" | tr ' ' ',')"
    log "RECOVERY (surgical): restarting Nav2/SLAM for [$*] only — other robots keep running (window ${window}s)"
    # Sim-side faults first: a dead scan republisher means '{ns}/map' can never
    # appear, however often Nav2 is restarted.
    # shellcheck disable=SC2086  # word-splitting of "$*" is intended
    ensure_scan_republishers $*
    for ns in $*; do
        robot_kill "$ns" || _SURGICAL_BLOCKED=1
    done
    if (( _SURGICAL_BLOCKED )); then
        log "ERROR: the old processes for [$*] could not be stopped — NOT relaunching:"
        log "       a second stack with duplicate node names cannot complete a bring-up."
        return 1
    fi
    # Let the killed processes' DDS discovery entries expire first: a client that
    # asks a dead service gets no reply, and that is what used to wedge the gate
    # on a robot that had already recovered (nav2.log 'failed to send response to
    # /robot_3/bt_navigator/get_state (timeout)', 2026-09-23).
    sleep "$SETTLE_AFTER_KILL"
    start_bg ros2 launch swarm_basics nav2_slam_all.launch.py \
        bt:="$BT" robots:="$comma" >>"$LOG_DIR/nav2.log" 2>&1
    NAV2_PIDS+=("${PIDS[${#PIDS[@]}-1]}")
    log "Waiting for Nav2 lifecycle nodes to become active (timeout ${window}s, guarded)..."
    run_gate "$window"
}

full_restart() {  # full_restart -> 0 if the gate passes
    local pid
    log "RECOVERY (full): restarting the WHOLE Nav2/SLAM stack (timeout ${NAV2_RETRY_TIMEOUT}s)..."
    for pid in "${NAV2_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    done
    pkill -KILL -f 'slam_toolbox' 2>/dev/null || true
    pkill -KILL -f 'nav2_' 2>/dev/null || true
    sleep 5   # let the process groups die AND the DDS traffic settle
    # NB (2026-09-18): do NOT clear /dev/shm here — the sim (gazebo, bridges,
    # robot_state_publisher) is still alive and deleting its SHM transport
    # files breaks TF/latched delivery for the freshly started Nav2 stack
    # (observed: all robots stuck on 'base_footprint does not exist').
    # Stale files are cleared pre-run (cleanup trap) and by stop_sim.sh.
    start_bg ros2 launch swarm_basics nav2_slam_all.launch.py bt:="$BT" >>"$LOG_DIR/nav2.log" 2>&1
    NAV2_PIDS+=("${PIDS[${#PIDS[@]}-1]}")
    log "Waiting for Nav2 lifecycle nodes to become active (timeout ${NAV2_RETRY_TIMEOUT}s, guarded)..."
    run_gate "$NAV2_RETRY_TIMEOUT"
}

: >"$LOG_DIR/nav2.log"   # fresh log; the probe only reacts to NEW output
ready=false

if [[ "$BRINGUP" == "sequential" ]]; then
    log "Bring-up: SEQUENTIAL — ${#ROBOTS[@]} stacks one at a time (${PER_ROBOT_TIMEOUT}s window, up to 3 attempts each)"
    ready=true
    for ns in "${ROBOTS[@]}"; do
        ok=false
        for attempt in 1 2 3; do
            launch_nav2_subset "$ns"
            log "Waiting for $ns to activate (attempt ${attempt}/3, window ${PER_ROBOT_TIMEOUT}s, guarded)..."
            if run_gate_single "$ns" "$PER_ROBOT_TIMEOUT"; then
                ok=true
                break
            fi
            log "RECOVERY: $ns did not activate — killing its stack and retrying"
            robot_kill "$ns" || log "WARNING: some $ns processes survived SIGKILL"
            ensure_scan_republishers "$ns"
            log "  settling ${SETTLE_AFTER_KILL}s before the next attempt"
            sleep "$SETTLE_AFTER_KILL"
        done
        if ! $ok; then
            log "ERROR: $ns never became ready after 3 attempts — aborting this boot"
            ready=false
            break
        fi
        log "$ns: ready"
    done
    if $ready; then
        # The viz-only map connectors normally come from the all-robot launch;
        # subset launches skip them (two publishers on one frame duplicate TFs),
        # so create them once here for the RViz overview.
        log "Creating viz map connectors (RViz overview only)"
        start_bg ros2 launch swarm_basics nav2_slam_all.launch.py \
            bt:="$BT" only_connectors:=true >>"$LOG_DIR/nav2.log" 2>&1
        NAV2_PIDS+=("${PIDS[${#PIDS[@]}-1]}")
    fi
else
    log "Bring-up: PARALLEL — every stack at once (legacy; SWARM_BRINGUP=parallel)"
    launch_nav2_all
    log "Waiting for Nav2 lifecycle nodes to become active (timeout ${READY_TIMEOUT}s, early stall-exit ${STALL_DETECT_SECS}s, guarded)..."
    run_gate "$READY_TIMEOUT" && ready=true

for _round in 1 2; do
    if ! $ready; then
        MISSING="$(failed_robots)"
        [[ -n "$MISSING" ]] || MISSING="${ROBOTS[*]}"
        NOSCAN="$(failed_noscan_robots)"
        if [[ -n "$NOSCAN" ]]; then
            log "DIAGNOSIS: [$NOSCAN] produced no LiDAR scan — sim-side symptom, not a Nav2 fault"
        fi
        # Round 2 gets a longer window: re-killing a robot whose SLAM needed a
        # few more seconds just re-restarted it and burned the whole attempt.
        if (( _round == 1 )); then
            SURGICAL_WINDOW="$SURGICAL_RETRY_TIMEOUT"
        else
            SURGICAL_WINDOW="$SURGICAL_RETRY_TIMEOUT_LONG"
        fi
        # shellcheck disable=SC2086  # word-splitting of "$MISSING" is intended
        if restart_subset $MISSING; then
            ready=true
        elif (( _SURGICAL_BLOCKED )); then
            break   # old processes refused to die: only a full restart can help
        fi
    fi
done

if ! $ready; then
    full_restart && ready=true
fi

if ! $ready; then
    # FINAL LIGHT RUNG (2026-09-23): the laggard is created DURING concurrent
    # bring-up, so a full restart — which brings every stack up at once again —
    # tends to re-create it.  One single-stack restart against robots that are
    # already healthy is far lighter, so try that once more with a long window
    # before throwing the attempt away (only when a strict subset failed: if
    # NOTHING came up, another surgical round cannot help).
    MISSING="$(failed_robots)"
    [[ -n "$MISSING" ]] || MISSING="${ROBOTS[*]}"
    read -r -a _MISSING_ARR <<<"$MISSING"
    if (( ${#_MISSING_ARR[@]} < ${#ROBOTS[@]} )); then
        log "RECOVERY (final, light): laggard [$MISSING] vs ${#ROBOTS[@]}-${#_MISSING_ARR[@]} healthy robot(s)"
        SURGICAL_WINDOW="$SURGICAL_RETRY_TIMEOUT_FINAL"
        # shellcheck disable=SC2086
        restart_subset $MISSING && ready=true
    fi
fi
fi

if ! $ready; then
    # FAIL FAST (2026-09-13): even a fresh FULL stack did not come up — abort
    # and let the batch retry the cell with a full fresh boot instead of idling.
    log "ERROR: bring-up failed — Nav2 not active for: ${ROBOTS[*]}"
    log "Aborting this run NOW (fail fast). Diagnostics: $LOG_DIR/nav2.log"
    cleanup
    exit 3
fi

# Gate passed -> tell the human detectors they may load YOLO now.
#
# With YOLO_WAIT_GATE=1 (the default) each detector idles without a
# model, CUDA context or inference until this fires, so the 5 x (torch import +
# CUDA context + model load) burst can never land inside bring-up or a recovery
# window — which is what a fixed --detector-delay kept doing (observed firing
# mid-full-restart at 200 s and mid-surgical-restart at 300 s on 2026-09-23).
# Detections are only needed once goals are being sent, i.e. from here on.
if [[ "${YOLO_WAIT_GATE:-0}" == "1" ]]; then
    # Gated detectors idle for free (no model, no CUDA context, a few hundred KB
    # of RAM), so there is no reason to start them after the goal phase has
    # begun: clamp the start delay so they are always alive when the gate
    # arrives, and human detection covers the whole run.
    # FLOAT-SAFE (2026-09-24): SWARM_DETECTOR_DELAY_S may be set as a decimal
    # (e.g. "200.0"), which bash arithmetic cannot evaluate — the old
    # `(( 200.0 > 30 ))` failed silently, so the clamp never fired, the
    # detectors started at +200 s (after the latched publisher had exited),
    # missed /swarm_ready and left the first minutes of the tour without human
    # data (run 2026-09-24 11:55).
    # DEFAULT-SAFE (2026-09-25): a direct run may not set
    # SWARM_DETECTOR_DELAY_S at all; under `set -u` the bare expansion aborted
    # the whole script right after bring-up.
    DET_DELAY_INT="${SWARM_DETECTOR_DELAY_S:-0}"
    DET_DELAY_INT="${DET_DELAY_INT%%.*}"
    if [[ "$DET_DELAY_INT" =~ ^[0-9]+$ ]] && (( DET_DELAY_INT > 30 )); then
        log "Detector start delay ${SWARM_DETECTOR_DELAY_S}s -> 20s (gated detectors idle for free; keeps human data valid from the first goal)"
        export SWARM_DETECTOR_DELAY_S=20
    fi
    log "Signalling human detectors to load YOLO (/swarm_ready, LATCHED)"
    # LATched (transient_local) + repeated: the detectors may start AFTER this
    # signal (they are delayed by SWARM_DETECTOR_DELAY_S), and a one-shot
    # volatile message is only seen by subscribers that already matched.  A
    # latched publisher hands the last value to every later subscriber — a
    # missed gate used to mean "human_detected=0 for the entire run"
    # (2026-09-23 11:44: all four detectors idled through the whole goal phase).
    ( timeout 30 ros2 topic pub -r 2 --qos-durability transient_local \
        /swarm_ready std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true ) &
    # DATA-INTEGRITY WATCH: say in the run log whether every detector actually
    # loaded.  "No detections" and "no humans in view" are otherwise
    # indistinguishable after the fact.
    (
        sleep 60
        missing=()
        for ns in "${ROBOTS[@]}"; do
            grep -qF "[${ns}.yolo_human_processor]: Loading YOLO model" "$LOG_DIR/sim.log" \
                || missing+=("$ns")
        done
        if (( ${#missing[@]} )); then
            log "WARNING: human detector(s) never loaded YOLO: ${missing[*]} — human_detected will stay 0 for them (see $LOG_DIR/sim.log)"
        else
            log "Human detectors loaded YOLO for all robots (human data valid)"
        fi
    ) &
fi
log "READY — Nav2 active for: ${ROBOTS[*]}"

# ------------------------------- 4) goals -----------------------------------
case "$GOALS" in
    random) GOALS_LAUNCH=random_goals_all.launch.py ;;
    fixed)  GOALS_LAUNCH=fixed_goals_all.launch.py ;;
    walk)   GOALS_LAUNCH=random_walk_all.launch.py ;;
    none)   GOALS_LAUNCH="" ;;
esac

# The detectors load their YOLO models the instant the gate is published — i.e.
# at exactly the moment the goal node used to start.  That 4x (torch import +
# CUDA context + model load) burst starved the Nav2 action servers: the 11:59 run
# logged 13 'no goal ACCEPT response after 10s' warnings, accepted only 2 goals
# and completed none.  Give the burst a short head start before sending goals.
GOALS_DELAY="${SWARM_GOALS_DELAY_S:-30}"
if [[ -n "$GOALS_LAUNCH" && "$ready" == "true" ]]; then
    if (( GOALS_DELAY > 0 )); then
        log "Starting goal node in ${GOALS_DELAY}s (lets the detector model loads settle): ros2 launch swarm_basics $GOALS_LAUNCH"
        start_bg bash -c "sleep ${GOALS_DELAY}; exec ros2 launch swarm_basics $GOALS_LAUNCH" \
            >"$LOG_DIR/goals.log" 2>&1
    else
        log "Starting goal node: ros2 launch swarm_basics $GOALS_LAUNCH"
        start_bg ros2 launch swarm_basics "$GOALS_LAUNCH" >"$LOG_DIR/goals.log" 2>&1
    fi
elif [[ -n "$GOALS_LAUNCH" ]]; then
    log "Goals NOT started (Nav2 not ready). Run manually: ros2 launch swarm_basics $GOALS_LAUNCH"
fi

# --------------------------- 5) auto-stop timer -----------------------------
# Graceful shutdown after RUN_TIME seconds of DRIVING (default 300 = 5 min).
# The goal node starts GOALS_DELAY after the gate, so the countdown adds that
# offset — RUN_TIME means driving time, not time-since-READY (2026-10-12;
# earlier runs counted from READY and got ~30 s less driving).
# Sends SIGINT to this script, which runs the exact same cleanup as Ctrl+C, so
# every logger flushes its summary and all CSVs/PNGs are saved.
TIMER_OFFSET=0
[[ -n "$GOALS_LAUNCH" && "$ready" == "true" ]] && TIMER_OFFSET="$GOALS_DELAY"
if (( RUN_TIME > 0 )); then
    log "Auto-stop timer armed: shutdown after ${RUN_TIME}s of driving (+${TIMER_OFFSET}s goal-node delay; disable: --run-time 0)."
    (
        sleep "$(( TIMER_OFFSET + RUN_TIME ))"
        printf '[%s] Run time (%ss) elapsed — shutting down and saving data.\n' \
            "$(date +%H:%M:%S)" "$RUN_TIME"
        kill -INT "$$"
    ) &
    WATCHDOG_PID=$!
fi

# ------------------------------- keep alive ---------------------------------
log "All processes running. Send goals from another terminal, or pass --goals."
log "Press Ctrl+C to shut everything down early."
wait
