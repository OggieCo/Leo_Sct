#!/usr/bin/env bash
# =============================================================================
# auto_retry_when_pending.sh — unattended retry companion for run_matrix.sh
#
# Why: run_matrix.sh runs ONE pass and does NOT retry failed cells within the
# same invocation (it exits with e.g. "Pass 1 finished — 29/30"). This watcher
# waits until the current pass is done, then re-invokes run_matrix.sh while
# cells are still pending. Each invocation resumes from completed.tsv, so it
# only re-runs the missing cells and stays in the SAME batch (no rollover:
# rollover only happens when completed.tsv is already full at a pass start).
#
# Stop conditions:
#   * all 30 cells completed -> exit (never triggers a new pass by itself)
#   * MAX_NO_PROGRESS consecutive invocations without a new completed cell
#     (i.e. the failure looks persistent rather than a one-off boot flake)
#
# Logs:    scripts/batch_logs/matrix/auto_retry.log      (watcher)
#          scripts/batch_logs/matrix/auto_retry_run.log  (matrix output)
# Stop it: pkill -f auto_retry_when_pending
# =============================================================================
set -u

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="$SCRIPTS_DIR/batch_logs/matrix"
TOTAL="$(( $(cat "$STATE_DIR/.total_cells" 2>/dev/null || echo 30) ))"  # from run_matrix.sh (42 for the scaled matrix); fallback 30 = old matrix
MAX_NO_PROGRESS=4     # consecutive invocations without progress before giving up
POLL=60               # seconds between "is the matrix still running" checks

log() { printf '[%s] auto-retry: %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$STATE_DIR/auto_retry.log"; }

mkdir -p "$STATE_DIR"
log "watcher started (pid $$) — waiting for run_matrix.sh to finish ..."

# 1) wait for the currently running pass to end
while pgrep -f "[r]un_matrix\.sh" >/dev/null; do sleep "$POLL"; done
sleep 20
log "matrix stopped — checking pending cells"

# 2) retry loop: re-invoke the matrix while cells are missing
noprog=0
while :; do
    TOTAL="$(( $(cat "$STATE_DIR/.total_cells" 2>/dev/null || echo "$TOTAL") ))"   # re-read: pass may use a different scope
    done_n=$(wc -l < "$STATE_DIR/completed.tsv" | tr -d ' ')
    if (( done_n >= TOTAL )); then
        log "all $TOTAL cells completed — nothing to retry. Exiting."
        break
    fi
    log "pending: $((TOTAL - done_n)) cell(s) — invoking run_matrix.sh (completed=$done_n)"
    bash "$SCRIPTS_DIR/run_matrix.sh" >> "$STATE_DIR/auto_retry_run.log" 2>&1 || true
    done_n2=$(wc -l < "$STATE_DIR/completed.tsv" | tr -d ' ')
    if (( done_n2 > done_n )); then
        noprog=0
        log "progress: completed $done_n -> $done_n2"
    else
        noprog=$((noprog + 1))
        log "no progress in this invocation ($noprog/$MAX_NO_PROGRESS)"
        if (( noprog >= MAX_NO_PROGRESS )); then
            log "giving up after $MAX_NO_PROGRESS attempts — failure looks persistent."
            log "check: $STATE_DIR/batch.log and results/coverage_logs/_failed_runs/"
            break
        fi
    fi
    sleep 30
done

log "watcher exiting"
