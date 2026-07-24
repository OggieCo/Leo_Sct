#!/usr/bin/env python3
"""
Recreate the coverage_plotter visualization from saved CSV data.
Usage:  python3 plot_coverage_from_csv.py <run_folder> [save_path]
Example:
    python3 plot_coverage_from_csv.py ../results/coverage_logs/run_2026-07-21_10-13-42/
"""

import sys
import csv
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ===== COLOUR CONFIG — change these to customise your plots =====
COLOR_VISITED = 'green'     # visited grid cells (e.g. 'yellow', '#ffcc00')
COLOR_EMPTY   = 'red'      # unvisited grid cells
COLOR_TRAJ    = 'blue'     # robot trajectory lines
GRID_ALPHA    = 0.5        # transparency of grid cells (0=transparent, 1=solid)
TRAJ_LW       = 0.8        # trajectory line width
# ================================================================


def load_trajectories(csv_path):
    """Load trajectories.csv -> {robot_id: [(x, y, yaw), ...]}"""
    traj = {}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            robot = row['robot_id']
            if robot not in traj:
                traj[robot] = []
            traj[robot].append((
                float(row['x']),
                float(row['y']),
                float(row['yaw']),
            ))
    return traj


def load_coverage_grid(csv_path):
    """Load coverage_final.csv -> set of visited (cell_x, cell_y)"""
    visited = set()
    cells = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            cx, cy = int(row['cell_x']), int(row['cell_y'])
            cells.append((cx, cy))
            if int(row['visited']):
                visited.add((cx, cy))
    return cells, visited


def plot_coverage(run_dir, save_path=None):
    """Replicate the coverage_plotter visualization from CSV files."""
    run_dir = Path(run_dir)

    # Load data
    traj = load_trajectories(run_dir / 'trajectories.csv')
    cells, visited = load_coverage_grid(run_dir / 'coverage_final.csv')

    # Determine bounds from cell list
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    env_min, env_max = min(xs), max(xs) + 1  # +1 because cells are 1m wide

    # Create plot
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Multi-Robot Global Coverage (from CSV)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.set_xlim(env_min, env_max)
    ax.set_ylim(env_min, env_max)

    # Draw grid cells
    for cx, cy in cells:
        color = COLOR_VISITED if (cx, cy) in visited else COLOR_EMPTY
        rect = Rectangle((cx, cy), 1.0, 1.0,
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
    ax.text(0.05, 0.95,
            f"Visited {n_visited}/{total} cells ({n_visited/total*100:.1f}%)",
            transform=ax.transAxes, fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none'))

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved plot to {save_path}")

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    run_dir = sys.argv[1]
    save = sys.argv[2] if len(sys.argv) > 2 else None
    plot_coverage(run_dir, save)
