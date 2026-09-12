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
#   scripts/start_experiment.sh --bt no                  # pure Nav2 baseline
#   scripts/start_experiment.sh --bt reactive            # reactive BT
#   scripts/start_experiment.sh --bt ai                  # BT + LLM layer
#   scripts/start_experiment.sh --bt ai --goals random   # + auto-start goals
#   scripts/start_experiment.sh --bt no --headless       # no GUI (batch runs)
#
# Options:
#   --bt no|reactive|ai     Behavior layer (required)
#   --llm true|false        Start llm_planner (default: true only for --bt ai)
#   --detector yolo|depth   Human detector backend (default: yolo)
#   --headless              Run Gazebo without GUI (auto-runs, no Play button)
#   --goal / --goals ...    none (default) | random | fixed | walk
#   --world PATH            Override the simulation world (sim_world:=PATH)
#   --ready-timeout SEC     Max wait for Nav2 to become active (default 240)
#   --run-time SEC          Graceful auto-stop N seconds after READY
#                           (default 300 = 5 min; 0 disables the timer)
#
# Press Ctrl+C to shut everything down (also saves all logs).
# =============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ------------------------------- defaults -----------------------------------
BT=""
LLM=""
DETECTOR="yolo"
HEADLESS="false"
WORLD=""
GOALS="none"
READY_TIMEOUT=240
RUN_TIME=300

# ------------------------------- helpers ------------------------------------
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,34p' "$0" | cut -c3-
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
        -h|--help)       usage 0 ;;
        *)               die "Unknown argument: $1 (try --help)" ;;
    esac
done

[[ -n "$BT" ]] || { echo "Missing required --bt argument." >&2; usage 1; }
case "$BT" in
    no|reactive|ai) ;;
    *) die "--bt must be one of: no, reactive, ai" ;;
esac
case "$GOALS" in
    none|random|fixed|walk) ;;
    *) die "--goals must be one of: none, random, fixed, walk" ;;
esac
[[ "$RUN_TIME" =~ ^[0-9]+$ ]] || die "--run-time must be a non-negative integer (seconds)"

# LLM default: only the 'ai' tree consumes it.
if [[ -z "$LLM" ]]; then
    if [[ "$BT" == "ai" ]]; then LLM="true"; else LLM="false"; fi
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

# Ignition CLI (Fortress ships 'ign'; newer Gazebo releases use 'gz').
IGN=""
if command -v ign >/dev/null 2>&1; then IGN="ign"
elif command -v gz >/dev/null 2>&1; then IGN="gz"
fi

# ------------------------------- logs ---------------------------------------
LOG_DIR="$SCRIPT_DIR/startup_logs/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$LOG_DIR"

# ------------------------------- cleanup ------------------------------------
PIDS=()
WATCHDOG_PID=""
cleanup() {
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
    log "Stopped. Logs: $LOG_DIR"
}
trap 'cleanup; exit 0' INT TERM

# ------------------------------- sanity check -------------------------------
if [[ -n "$IGN" ]] && $IGN service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/control"; then
    die "A Gazebo world '${WORLD_NAME}' is already running — stop it first."
fi
if pgrep -f 'ros2 launch swarm_basics' >/dev/null 2>&1 \
   || pgrep -f '/swarm_basics/lib/swarm_basics/' >/dev/null 2>&1; then
    die "Leftover swarm_basics processes detected from a previous run. Clean up first:
    pkill -INT -f 'ros2_ws/install'; pkill -INT -f 'ros_gz_bridge'; pkill -INT -f 'slam_toolbox'; pkill -INT -f 'nav2_'; sleep 3
    pgrep -af 'ros2_ws/install|ros_gz_bridge|slam_toolbox|nav2_' || echo all-clean"
fi

log "Mode: bt=$BT  llm=$LLM  detector=$DETECTOR  headless=$HEADLESS  goals=$GOALS"
[[ -n "$HAVE_SETSID" ]] || log "WARNING: 'setsid' not found — shutdown may not reach all child processes."
[[ -n "$HAVE_ENV_RESET" ]] || log "WARNING: 'env --default-signal' unavailable — graceful SIGINT may not reach the stack; shutdown will escalate to SIGTERM/SIGKILL."

# ------------------------------- 1) Gazebo ----------------------------------
SIM_ARGS=(headless:="$HEADLESS" detector:="$DETECTOR" enable_llm:="$LLM" bt:="$BT")
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
log "Starting SLAM + Nav2: ros2 launch swarm_basics nav2_slam_all.launch.py bt:=$BT"
start_bg ros2 launch swarm_basics nav2_slam_all.launch.py bt:="$BT" >"$LOG_DIR/nav2.log" 2>&1

# ------------------------------- 3) readiness gate --------------------------
log "Waiting for Nav2 lifecycle nodes to become active (timeout ${READY_TIMEOUT}s)..."
ready=false
deadline=$((SECONDS + READY_TIMEOUT))
while (( SECONDS < deadline )); do
    all=1
    for ns in "${ROBOTS[@]}"; do
        state="$(ros2 lifecycle get "/${ns}/bt_navigator" 2>/dev/null || true)"
        [[ "$state" == active* ]] || all=0
    done
    if (( all )); then ready=true; break; fi
    sleep 3
done

if $ready; then
    log "READY — Nav2 active for: ${ROBOTS[*]}"
else
    log "WARNING: readiness timeout — Nav2 not active yet. Check $LOG_DIR/nav2.log"
fi

# ------------------------------- 4) goals -----------------------------------
case "$GOALS" in
    random) GOALS_LAUNCH=random_goals_all.launch.py ;;
    fixed)  GOALS_LAUNCH=fixed_goals_all.launch.py ;;
    walk)   GOALS_LAUNCH=random_walk_all.launch.py ;;
    none)   GOALS_LAUNCH="" ;;
esac

if [[ -n "$GOALS_LAUNCH" && "$ready" == "true" ]]; then
    log "Starting goal node: ros2 launch swarm_basics $GOALS_LAUNCH"
    start_bg ros2 launch swarm_basics "$GOALS_LAUNCH" >"$LOG_DIR/goals.log" 2>&1
elif [[ -n "$GOALS_LAUNCH" ]]; then
    log "Goals NOT started (Nav2 not ready). Run manually: ros2 launch swarm_basics $GOALS_LAUNCH"
fi

# --------------------------- 5) auto-stop timer -----------------------------
# Graceful shutdown N seconds after the stack is READY (default 5 min).
# Sends SIGINT to this script, which runs the exact same cleanup as Ctrl+C, so
# every logger flushes its summary and all CSVs/PNGs are saved.
if (( RUN_TIME > 0 )); then
    log "Auto-stop timer armed: shutdown in ${RUN_TIME}s (disable: --run-time 0)."
    (
        sleep "$RUN_TIME"
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
