#!/usr/bin/env bash
# =============================================================================
# purge_resolved_failures.sh — delete quarantined failed runs once their cell
# has been completed successfully (by a later retry).
#
# A quarantine entry is named  '<MODE>_<scenario>_<batch>_run_<ts>'
# (MODE: Nav2|BT|BT_LLM — written by run_matrix.sh's quarantine_run()).
# An entry is purged when:
#   * the CURRENT batch (scripts/batch_logs/matrix/.current_batch) matches the
#     batch recorded in the folder name, AND
#   * the cell (arm|scenario|1) appears in completed.tsv — i.e. the failure is
#     history and the retry produced the authoritative run folder.
# Everything else is left alone (unresolved failures stay for the retry).
#
# Every removal is appended to batch.log and to _failed_runs/PURGED.tsv so the
# thesis can still state how many boot failures were discarded.
# Safe to run anytime; also invoked at the end of every pass from
# run_matrix.sh (wired in the batch_3 edit, 2026-09-14).
# =============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="/root/ros2_ws/src/results/coverage_logs"
QUARANTINE_DIR="$BASE_DIR/_failed_runs"
STATE_DIR="$SCRIPT_DIR/batch_logs/matrix"
COMPLETED="$STATE_DIR/completed.tsv"
GEN="$SCRIPT_DIR/../worlds/scenario_generator.py"
BATCH="$(cat "$STATE_DIR/.current_batch" 2>/dev/null || echo batch_1)"

logline() { printf '[%s] purge: %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$STATE_DIR/batch.log"; }

[[ -d "$QUARANTINE_DIR" ]] || { echo "no $QUARANTINE_DIR — nothing to purge"; exit 0; }
[[ -f "$COMPLETED" ]]      || { echo "no completed.tsv — nothing to purge"; exit 0; }

# MODE (folder prefix) -> arm (cell key); mirror of run_matrix.sh's mode_of()
declare -A ARM_OF=( [Nav2]=no [BT]=reactive [BT_LLM]=ai )

purged=0
for entry in "$QUARANTINE_DIR"/*/; do
    [[ -d "$entry" && "$entry" == "$QUARANTINE_DIR"/* ]] || continue
    base="$(basename "$entry")"
    for mode in Nav2 BT BT_LLM; do
        [[ "$base" == "${mode}_"* ]] || continue
        rest="${base#${mode}_}"
        matched=0
        for n in 1 2 3 4 5 6 7 8 9 10; do
            name="$(python3 "$GEN" --scenario "$n" --print-name 2>/dev/null || true)"
            [[ -n "$name" ]] || continue
            [[ "$rest" == "${name}_"* ]] || continue
            tail_part="${rest#${name}_}"                 # batch_N_run_<ts>
            batch="${tail_part%%_run_*}"
            arm="${ARM_OF[$mode]}"
            if [[ "$batch" == "$BATCH" ]] && grep -qxF "${arm}|${n}|1" "$COMPLETED"; then
                if rm -rf -- "$entry"; then
                    logline "removed resolved failed run: $base (cell ${arm}|${n}|1 completed in $batch)"
                    printf '%s\t%s\n' "$(date +%Y-%m-%dT%H:%M:%S)" "$base" >> "$QUARANTINE_DIR/PURGED.tsv"
                    purged=$((purged+1))
                else
                    logline "WARN: could not remove $base"
                fi
            fi
            matched=1
            break
        done
        [[ $matched -eq 1 ]] && break
    done
done

echo "purge scan (batch $BATCH): $purged folder(s) removed"
exit 0
