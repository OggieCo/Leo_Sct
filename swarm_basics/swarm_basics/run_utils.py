"""Shared run-folder logic for coverage_plotter, social_event_logger and
llm_planner.

All nodes must write into the SAME per-run folder, inside a MODE subfolder:
  coverage_logs/BT_LLM/run_<timestamp>/  (LLM decision layer enabled)
  coverage_logs/BT/run_<timestamp>/      (pure reactive BT, no LLM)
The mode comes from the SWARM_RUN_MODE env var set by the launch (the
enable_llm arg), so LLM runs and pure-reactive runs are separated for the
thesis A/B comparison.

If each node computed its own timestamp independently they could start a
second apart and create two folders (observed: run_..._14-21-41 vs
run_..._14-21-42).  `get_run_dir()` reuses the most recently created run_
folder (within the same mode) if it is only a few seconds old, otherwise
creates a fresh one — so whichever node starts first creates the folder and
the others join it.
"""

import os
import time
from pathlib import Path

BASE_DIR = Path('/root/ros2_ws/src/results/coverage_logs')
_MODES = ('BT_LLM', 'BT')
# A run_ folder younger than this is considered "the current run".
_RECENT_SECONDS = 5.0


def _mode():
    """BT_LLM or BT from the SWARM_RUN_MODE env var (default LLM)."""
    mode = os.environ.get('SWARM_RUN_MODE', 'BT_LLM')
    return mode if mode in _MODES else 'BT_LLM'


def get_run_dir():
    """Return the shared Path for the current run (creates it if needed)."""
    root = BASE_DIR / _mode()
    root.mkdir(parents=True, exist_ok=True)

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
    if newest is not None and (now - newest_mtime) < _RECENT_SECONDS:
        return newest

    d = root / time.strftime('run_%Y-%m-%d_%H-%M-%S')
    d.mkdir(parents=True, exist_ok=True)
    return d
