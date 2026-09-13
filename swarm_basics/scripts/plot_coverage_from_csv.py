#!/usr/bin/env python3
"""
Recreate the coverage_plotter visualization from saved CSV data.

Works from the CSVs alone — coverage_final.csv is NOT required:
  * trajectories.csv    -> robot paths (visited cells are derived from poses)
  * events.csv          -> CELL_VISITED events (extra fallback)
  * coverage_final.csv  -> used when present (exact grid snapshot)

Usage:  python3 plot_coverage_from_csv.py <run_folder> [save_path]
Example:
    python3 plot_coverage_from_csv.py ../results/coverage_logs/run_2026-07-21_10-13-42/
If [save_path] is omitted the PNG is written to <run_folder>/coverage_map.png.
"""

import sys
import csv
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')          # headless: render to a file, never open a window
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ===== COLOUR CONFIG — change these to customise your plots =====
COLOR_VISITED = 'green'     # visited grid cells (e.g. 'yellow', '#ffcc00')
COLOR_EMPTY   = 'red'      # unvisited grid cells
COLOR_TRAJ    = 'blue'     # robot trajectory lines
GRID_ALPHA    = 0.5        # transparency of grid cells (0=transparent, 1=solid)
TRAJ_LW       = 1.5        # trajectory line width (matches coverage_plotter default)
# ================================================================

# Same grid as coverage_plotter.py: 1 m cells over [-7, 7) in x and y.
ENV_MIN, ENV_MAX = -7, 7
GRID_SIZE = 1.0


def load_trajectories(csv_path):
    """Load trajectories.csv -> {robot_id: [(x, y, yaw), ...]}"""
    traj = {}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                robot = row['robot_id']
                point = (float(row['x']), float(row['y']), float(row['yaw']))
            except (TypeError, ValueError):
                continue  # tolerate a truncated final line from a hard-killed run
            traj.setdefault(robot, []).append(point)
    return traj


def load_coverage_grid(csv_path):
    """Load coverage_final.csv -> (cells, visited); empty result if absent."""
    visited = set()
    cells = []
    if not Path(csv_path).exists():
        return cells, visited
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            cx, cy = int(row['cell_x']), int(row['cell_y'])
            cells.append((cx, cy))
            if int(row['visited']):
                visited.add((cx, cy))
    return cells, visited


def load_cell_events(csv_path):
    """Load CELL_VISITED rows from events.csv -> set of visited (cell_x, cell_y)."""
    visited = set()
    if not Path(csv_path).exists():
        return visited
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get('event_type') != 'CELL_VISITED':
                continue
            try:
                _, cx, cy = (row.get('details') or '').split('_')  # 'cell_<x>_<y>'
                visited.add((int(cx), int(cy)))
            except ValueError:
                continue
    return visited


def cells_from_trajectories(traj):
    """Grid cells covered by any recorded pose (same rule as coverage_plotter)."""
    visited = set()
    for pts in traj.values():
        for x, y, _yaw in pts:
            cx, cy = math.floor(x), math.floor(y)
            if ENV_MIN <= cx < ENV_MAX and ENV_MIN <= cy < ENV_MAX:
                visited.add((cx, cy))
    return visited


def plot_coverage(run_dir, save_path=None):
    """Replicate the coverage_plotter visualization from CSV files."""
    run_dir = Path(run_dir)
    if save_path is None:
        save_path = run_dir / 'coverage_map.png'

    # Load data (coverage_final.csv optional; trajectories.csv is required)
    traj = load_trajectories(run_dir / 'trajectories.csv')
    cells, visited_grid = load_coverage_grid(run_dir / 'coverage_final.csv')
    if not cells:
        cells = [(x, y) for x in range(ENV_MIN, ENV_MAX)
                        for y in range(ENV_MIN, ENV_MAX)]

    # Visited cells, in order of exactness:
    #  1. coverage_final.csv      — exact snapshot written by the plotter
    #  2. events.csv CELL_VISITED — logged live by the same check (+-0 cells)
    #  3. trajectory-derived cells — last resort only: the poses in the CSV are
    #     rounded to 6 decimals, so a pose near a cell boundary can fall into a
    #     neighbouring cell here and inflate the count by ~1 cell.
    visited = set(visited_grid) | load_cell_events(run_dir / 'events.csv')
    if not visited:
        visited = cells_from_trajectories(traj)
    visited &= set(cells)

    # Create plot
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Multi-Robot Global Coverage (from CSV)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.set_xlim(ENV_MIN, ENV_MAX)
    ax.set_ylim(ENV_MIN, ENV_MAX)

    # Draw grid cells
    for cx, cy in cells:
        color = COLOR_VISITED if (cx, cy) in visited else COLOR_EMPTY
        rect = Rectangle((cx, cy), GRID_SIZE, GRID_SIZE,
                         facecolor=color, edgecolor='black', alpha=GRID_ALPHA)
        ax.add_patch(rect)

    # Draw trajectories
    for robot, pts in traj.items():
        if len(pts) > 1:
            xs_t, ys_t = zip(*[(p[0], p[1]) for p in pts])
            ax.plot(xs_t, ys_t, label=robot, linewidth=TRAJ_LW)

    if traj:
        ax.legend(loc="upper right", fontsize="small")

    # Coverage stats
    total = len(cells)
    n_visited = len(visited)
    pct = (n_visited / total * 100) if total else 0.0
    ax.text(0.05, 0.95,
            f"Visited {n_visited}/{total} cells ({pct:.1f}%)",
            transform=ax.transAxes, fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none'))

    # No tight_layout(): coverage_plotter.py does not use it, so leaving it out
    # keeps both PNGs framed identically (same grid extent on the canvas).
    fig.savefig(save_path, dpi=150)
    # matplotlib writes RGBA PNGs; convert to RGB for VS Code's image preview.
    try:
        from PIL import Image
        Image.open(save_path).convert('RGB').save(save_path)
    except Exception:
        pass

    # Summary stats to stdout (recomputed from the CSVs)
    total_len = 0.0
    for pts in traj.values():
        for (x0, y0, _), (x1, y1, _) in zip(pts, pts[1:]):
            total_len += math.hypot(x1 - x0, y1 - y0)

    print(f"Saved plot to {save_path}")
    print(f"Visited {n_visited}/{total} cells ({pct:.1f}%)")
    print(f"Robots: {len(traj)}  |  total path length: {total_len:.1f} m")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    run_dir = sys.argv[1]
    save = sys.argv[2] if len(sys.argv) > 2 else None
    plot_coverage(run_dir, save)
