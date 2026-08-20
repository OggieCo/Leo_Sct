"""Shared run-folder logic for coverage_plotter and social_event_logger.

Both nodes must write into the SAME per-run folder
(/root/ros2_ws/src/results/coverage_logs/run_<timestamp>/).  If each node
computed its own timestamp independently they could start a second apart and
create two folders (observed: run_..._14-21-41 vs run_..._14-21-42).

`get_run_dir()` reuses the most recently created run_ folder if it is only a
few seconds old, otherwise creates a fresh one — so whichever node starts
first creates the folder and the other joins it.
"""

import time
from pathlib import Path

BASE_DIR = Path('/root/ros2_ws/src/results/coverage_logs')
# A run_ folder younger than this is considered "the current run".
_RECENT_SECONDS = 5.0


def get_run_dir():
    """Return the shared Path for the current run (creates it if needed)."""
    BASE_DIR.mkdir(parents=True, exist_ok=True)

    newest = None
    newest_mtime = -1.0
    for d in BASE_DIR.iterdir():
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

    d = BASE_DIR / time.strftime('run_%Y-%m-%d_%H-%M-%S')
    d.mkdir(parents=True, exist_ok=True)
    return d
