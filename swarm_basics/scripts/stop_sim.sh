#!/usr/bin/env bash
# =============================================================================
# stop_sim.sh — stop EVERYTHING from a simulation run (sim + Nav2 + SLAM + ...)
#
# Usage (inside the container):        bash scripts/stop_sim.sh
# Or from the host:                    docker exec humble_thesis_engine \
#                                        bash /root/ros2_ws/src/swarm_basics/scripts/stop_sim.sh
#
# Escalates INT -> TERM -> KILL until the process list is clean, clears stale
# FastDDS /dev/shm segments, and prints what it did.
#
# WHY THIS IS A FILE (do not inline these commands): when the same pkill
# patterns are typed into a `bash -lc "..."` command line, the invoking shell
# itself matches `pkill -f` and the cleanup kills its own session (incident:
# 2026-09-17 — docker exec shell got SIGTERM mid-cleanup, exit 143).  Running
# from a file keeps the patterns OUT of the invoking command line.
# =============================================================================
set -u

PATTERNS=(
    'ros2 launch swarm_basics'
    'ign gazebo'
    'swarm_basics/lib'
    'ros_gz_bridge'
    'slam_toolbox'
    'nav2_'
    'robot_state_publisher'
)

LOG_PATTERN="$(IFS='|'; echo "${PATTERNS[*]}")"

alive() { pgrep -f "$LOG_PATTERN" >/dev/null 2>&1; }

echo "=== before ==="
pgrep -af "$LOG_PATTERN" 2>/dev/null | head -10 || true

for wave in "INT:6" "TERM:4" "KILL:2"; do
    sig="${wave%%:*}"; wait_s="${wave##*:}"
    for p in "${PATTERNS[@]}"; do
        pkill -"$sig" -f "$p" 2>/dev/null || true
    done
    sleep "$wait_s"
    if alive; then
        echo "(still alive after $sig — escalating)"
    else
        break
    fi
done

# Clear stale FastDDS SHM state (segments, port files, event locks).
# 2026-09-18: '_port*' / '*_el' files slipped past the old patterns and broke
# SHM port init on later boots (spawners never received robot_description).
rm -f /dev/shm/*fastrtps* /dev/shm/*fastdds* /dev/shm/_port* /dev/shm/*_el 2>/dev/null

echo "=== after ==="
if alive; then
    echo "WARNING: processes still present:"
    pgrep -af "$LOG_PATTERN" 2>/dev/null | head -10 || true
    exit 1
else
    echo "all-clean"
fi
