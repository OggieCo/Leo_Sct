"""Shared run-folder logic for coverage_plotter, social_event_logger and
llm_planner.

All nodes must write into the SAME per-run folder, inside a MODE subfolder:
  coverage_logs/BT_LLM/run_<timestamp>/  (LLM decision layer enabled)
  coverage_logs/BT/run_<timestamp>/      (pure reactive BT, no LLM)
  coverage_logs/Nav2/run_<timestamp>/    (stock Nav2, no social layer)
  ...and with the batch tag in the NAME when running a scenario:
  coverage_logs/<MODE>/<name>/<batch>_run_<timestamp>/
  with <name> e.g. 2_rovers_3_humans and <batch> = batch_1, batch_2, ...
  (one data-collection campaign per batch: strategy / seed / batch_X).
  The batch comes from SWARM_BATCH (run_matrix.sh exports it per pass);
  manual runs without it JOIN the newest existing batch of that scenario.
  SWARM_RUN_DIR overrides everything for ad-hoc / GUI runs: the run folder
  is created directly inside that directory (plain run_<timestamp> name).
The mode comes from the SWARM_RUN_MODE env var set by the launch (the
enable_llm arg, overridden to Nav2 by bt:=stock), so the three conditions are
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

# Contact -> bump episode cooldown, SHARED by bump_counter, coverage_plotter and
# social_event_logger so all three agree 1:1 (one event per contact episode).
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


def _batch_name(root):
    """Batch tag used as a prefix of new run folder names under `root`.

    SWARM_BATCH (exported per pass by run_matrix.sh, e.g. 'batch_2') pins the
    data-collection campaign.  Without it a manual run joins the newest
    existing batch, or starts with batch_1."""
    name = os.environ.get('SWARM_BATCH', '').strip()
    if name:
        return ''.join(c if (c.isalnum() or c in '-_') else '_' for c in name)
    newest = None
    try:
        for d in root.iterdir():
            if not (d.is_dir() and d.name.startswith('batch_')):
                continue
            try:
                n = int(d.name.split('_', 1)[1].split('_', 1)[0])
            except (IndexError, ValueError):
                continue
            if newest is None or n > newest:
                newest = n
    except OSError:
        pass
    return f'batch_{newest}' if newest is not None else 'batch_1'


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
    override = os.environ.get('SWARM_RUN_DIR', '').strip()
    if override:
        # Ad-hoc / GUI runs ("random runs"): everything lands directly in that folder
        # with plain run_<timestamp> names, no MODE/scenario/ batch structure.
        root = Path(override)
        prefix = ''
    else:
        root = _scenario_root(BASE_DIR / _mode())
        prefix = _batch_name(root) + '_'
    root.mkdir(parents=True, exist_ok=True)

    rid = _active_run_id()
    if rid:
        # JOIN the folder whichever mode root already has it (the sim/Nav2 launch
        # creates it with the authoritative mode); only create it ourselves if nobody
        # did yet.
        if not override:
            for mode in _MODES:
                mode_root = _scenario_root(BASE_DIR / mode)
                found = sorted(p for p in mode_root.glob(f'*run_{rid}')
                               if p.is_dir())
                if found:
                    return found[0]
        d = root / f'{prefix}run_{rid}'
        d.mkdir(parents=True, exist_ok=True)
        return d

    # manual launch (no SWARM_RUN_ID): join the newest live run folder
    newest = None
    newest_mtime = -1.0
    for d in root.iterdir():
        if d.is_dir() and 'run_' in d.name:
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

    d = root / f'{prefix}run_{time.strftime("%Y-%m-%d_%H-%M-%S")}'
    d.mkdir(parents=True, exist_ok=True)
    return d
