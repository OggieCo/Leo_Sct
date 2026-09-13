"""Shared run-folder logic for coverage_plotter, social_event_logger and
llm_planner.

All nodes must write into the SAME per-run folder, inside a MODE subfolder:
  coverage_logs/BT_LLM/run_<timestamp>/  (LLM decision layer enabled)
  coverage_logs/BT/run_<timestamp>/      (pure reactive BT, no LLM)
  coverage_logs/Nav2/run_<timestamp>/    (stock Nav2, no social layer)
  ...and under a scenario-name level when running a seed scenario:
  coverage_logs/<MODE>/<name>/run_<timestamp>/   (e.g. 2_rovers_2_humans)
The mode comes from the SWARM_RUN_MODE env var set by the launch (the
enable_llm arg, overridden to Nav2 by bt:=no), so the three conditions are
separated for the thesis comparison.

start_experiment.sh exports SWARM_RUN_ID (its launch timestamp) before
starting anything, so EVERY node of a launch — including the goal drivers,
which it starts only after the Nav2 readiness gate ~30 s later — resolves to
the same run_<id> folder and all CSVs of a run land together.

For manual launches (no SWARM_RUN_ID) the previous heuristic stays: reuse the
most recently created run_ folder (within the same mode) if it is only a few
seconds old OR was written to seconds ago (a live run rewrites
social_state.csv at 1 Hz), otherwise create a fresh one — so whichever node
starts first creates the folder and the others join it.  Without the
"written seconds ago" check a late-starting node splintered the run
(observed: run_..._16-52-57 plus run_..._16-53-28 holding only
goal_stats.csv).
"""

import os
import time
from pathlib import Path

BASE_DIR = Path('/root/ros2_ws/src/results/coverage_logs')
_MODES = ('BT_LLM', 'BT', 'Nav2')
# A run_ folder younger than this is considered "the current run".
_RECENT_SECONDS = 5.0
# ...and a run_ folder with a file THIS young is a LIVE run (social_state.csv
# is rewritten at 1 Hz), so nodes started mid-run join it instead of
# splintering a second folder.
_ACTIVE_SECONDS = 60.0

# Contact -> bump episode cooldown, SHARED by bump_counter, coverage_plotter
# and social_event_logger so all three counters agree 1:1: one BUMP / one
# CONTACT_BEGIN+END pair per contact episode; a sustained press counts once,
# and only a real >= CONTACT_COOLDOWN_SEC separation re-arms the entity.
CONTACT_COOLDOWN_SEC = 3.0


def _mode():
    """BT_LLM, BT or Nav2 from the SWARM_RUN_MODE env var (default LLM)."""
    mode = os.environ.get('SWARM_RUN_MODE', 'BT_LLM')
    return mode if mode in _MODES else 'BT_LLM'


def _scenario_root(root):
    """Insert the scenario-name level when the run belongs to a scenario:
    coverage_logs/<MODE>/<name>/run_<timestamp>/ (SWARM_SCENARIO carries the
    name, e.g. '2_rovers_2_humans', exported by start_experiment.sh
    --scenario; SWARM_SEED is still read as a legacy fallback).  Runs without
    a scenario keep the old flat layout."""
    name = (os.environ.get('SWARM_SCENARIO', '')
            or os.environ.get('SWARM_SEED', '')).strip()
    if name:
        safe = ''.join(c if (c.isalnum() or c in '-_') else '_' for c in name)
        root = root / safe
    return root


def _active_run_id():
    """SWARM_RUN_ID exported by start_experiment.sh for this launch ('' when
    the node was started manually)."""
    rid = os.environ.get('SWARM_RUN_ID', '').strip()
    return ''.join(c if (c.isalnum() or c in '-_') else '_' for c in rid)


def _touched_recently(d, now):
    """True if any file inside `d` was written less than _ACTIVE_SECONDS ago
    (a live run keeps writing CSVs at 1 Hz; a finished one stops)."""
    try:
        for f in d.iterdir():
            if f.is_file() and (now - f.stat().st_mtime) < _ACTIVE_SECONDS:
                return True
    except OSError:
        pass
    return False


def get_run_dir():
    """Return the shared Path for the current run (creates it if needed)."""
    root = _scenario_root(BASE_DIR / _mode())
    root.mkdir(parents=True, exist_ok=True)

    rid = _active_run_id()
    if rid:
        # Deterministic: start_experiment.sh exported the run id before
        # launching anything, so every node — however late it starts (the
        # goal drivers wait for the Nav2 readiness gate) — resolves to the
        # same run_<id> folder, goal_stats.csv included.
        d = root / f'run_{rid}'
        d.mkdir(parents=True, exist_ok=True)
        return d

    newest = None
    newest_mtime = -1.0
    for d in root.iterdir():
        if d.is_dir() and d.name.startswith('run_'):
            try:
                m = d.stat().st_mtime
            except OSError:
                continue
            if m > newest_mtime:
                newest_mtime = m
                newest = d

    now = time.time()
    if newest is not None and ((now - newest_mtime) < _RECENT_SECONDS
                               or _touched_recently(newest, now)):
        return newest

    d = root / time.strftime('run_%Y-%m-%d_%H-%M-%S')
    d.mkdir(parents=True, exist_ok=True)
    return d
