#!/usr/bin/env bash
# =============================================================================
# runbook.sh — command cheat-sheet matching the SCALED matrix (2026-09-17)
#   2-5 rovers x 1-3 humans (+ obstacle variants), seed support, batch_3 state.
#
# Prints:
#   * every manual per-scenario command for the three arms
#   * seeded-run examples (SWARM_RUN_SEED)
#   * batch start / stop / resume / watch commands
#
# Usage (host or container):
#   bash scripts/runbook.sh                 # full sheet (14 scenarios x 3 arms)
#   bash scripts/runbook.sh --only 8        # just 4_rovers_2_humans
#   bash scripts/runbook.sh --only 3,6,12   # crowded columns only
# =============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEN="$SCRIPT_DIR/../worlds/scenario_generator.py"

ONLY=""
if [[ "${1:-}" == "--only" && -n "${2:-}" ]]; then
    ONLY="$2"
fi

scenario_numbers=()
if [[ -n "$ONLY" ]]; then
    IFS=',' read -r -a scenario_numbers <<< "$ONLY"
else
    scenario_numbers=(1 2 3 4 5 6 7 8 9 10 11 12 13 14)
fi

name_of() { python3 "$GEN" --scenario "$1" --print-name 2>/dev/null; }

cat <<'EOF'
===========================================================================
 RUNBOOK — scaled matrix (2-5 rovers x 1-3 humans)             state: batch_3
===========================================================================
 Numbers:  1:2r1h  2:2r2h  3:2r3h  4:3r1h  5:3r2h  6:3r3h  7:4r1h
           8:4r2h  9:4r3h 10:5r1h 11:5r2h 12:5r3h 13:3r+obst 14:5r+obst
 Notes:
   * run from the repo dir:   cd /root/ros2_ws/src/swarm_basics
   * 4-5 rovers boot slower — wait for "READY" (up to ~2-3 min)
   * add a seed for reproducible runs:  SWARM_RUN_SEED=1 <command>
   * recording from above: GUI runs only — move camera overhead, press the
     Video Recorder icon (top-right); mp4 lands in the launch directory
   * stop a running experiment: Ctrl+C   (CSVs are flushed either way)
   * stuck leftover sim? bash scripts/stop_sim.sh  (kills sim+Nav2+SLAM, INT->TERM->KILL)
   * old 1-rover scenarios no longer exist; numbering CHANGED (1 now = 2r1h)
   * logs per arm:  stock -> Nav2/   reactive -> BT/   ai -> BT_LLM/
===========================================================================
EOF

for arm in stock reactive ai; do
    case "$arm" in
        stock)    label='🟨 Nav2 (--bt stock)' ;;
        reactive) label='🟦 Nav2 + BT (--bt reactive)' ;;
        ai)       label='🟪 Nav2 + BT + LLM (--bt ai)' ;;
    esac
    echo
    echo "$label"
    for n in "${scenario_numbers[@]}"; do
        name="$(name_of "$n")"
        [[ -n "$name" ]] || continue
        printf './scripts/start_experiment.sh --scenario %s --bt %s --goals fixed\n' "$name" "$arm"
    done
done

cat <<'EOF'

===========================================================================
 SEEDED RUNS (reproducible; the same seed replays the same goal sequence)
   one:   SWARM_RUN_SEED=1 ./scripts/start_experiment.sh \
              --scenario 4_rovers_2_humans --bt ai --goals random
   three: for s in 1 2 3; do SWARM_RUN_SEED=$s ./scripts/start_experiment.sh \
              --scenario 4_rovers_2_humans --bt ai --goals random --run-time 300; done
   (in batch mode the seed is set automatically when REPEATS>1)
===========================================================================

===========================================================================
 AD-HOC / RANDOM RUNS — one flat folder, no MODE/scenario structure
   R=/root/ros2_ws/src/results/coverage_logs/random_runs
   SWARM_RUN_DIR=$R ./scripts/start_experiment.sh --scenario 4_rovers_2_humans --bt stock    --goals random
   SWARM_RUN_DIR=$R ./scripts/start_experiment.sh --scenario 4_rovers_2_humans --bt reactive --goals random
   SWARM_RUN_DIR=$R ./scripts/start_experiment.sh --scenario 4_rovers_2_humans --bt ai       --goals random
   -> every run creates $R/run_<timestamp>/ with all CSVs inside.
      Want per-arm separation? use $R/<arm> as SWARM_RUN_DIR (e.g. $R/ai).
      Use this for showcase/recording runs; campaign runs keep their
      MODE/scenario folders.
===========================================================================

===========================================================================
 BATCH — 3 arms x 14 scenarios = 42 cells   (REPEATS=1; REPEATS=10 = seeds)
   preview only:      DRY_RUN=1 bash scripts/run_matrix.sh
   start:             cd /root/ros2_ws/src/swarm_basics
                      nohup bash scripts/run_matrix.sh >/dev/null 2>&1 &
   watch:             tail -f scripts/batch_logs/matrix/batch.log
   stop (graceful):   pkill -INT -f run_matrix.sh
   resume:            cd /root/ros2_ws/src/swarm_basics && \
                      nohup bash scripts/run_matrix.sh >/dev/null 2>&1 &
   seeds:             edit REPEATS=10 in scripts/run_matrix.sh (seed = repeat #)
   scope:             edit SCENARIOS=(...) — crowded only: (3 6 9 12 14)
   state:             scripts/batch_logs/matrix  (batch_3; old matrix archived)
   health:            pgrep -af "ros2_ws/install|ign gazebo" || echo all-clean
===========================================================================
EOF
