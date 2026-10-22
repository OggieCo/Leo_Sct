#!/usr/bin/env python3
"""patch_coverage_from_path.py — recompute a run's coverage grid from the FULL
drawn trajectory instead of the raw odom sample cells.

Motivation (2026-10-09): one run's odom logger lost a ~210 s window during a
retry-heavy bring-up, so trajectories.csv contains a straight "ghost" segment
(spawn position -> true position) that cuts across cells which are not marked
visited. This script marks every cell that any logged segment passes through
(supercover, 0.02 m sampling), which makes the coverage map consistent with the
drawn paths.

Updates in place: coverage_final.csv (full rewrite),
visited_cells/coverage_pct in summary.csv (only these two rows change),
coverage_map.png (same style as coverage_plotter.save_final_plot).

Usage (inside the container, MPLBACKEND=Agg is set internally):
    python3 scripts/patch_coverage_from_path.py <run_dir>
"""
import csv
import math
import os
import sys
from collections import defaultdict

GRID = [(x, y) for x in range(-7, 7) for y in range(-7, 7)]


def cell(x, y):
    ix, iy = math.floor(x), math.floor(y)
    return (ix, iy) if -7 <= ix < 7 and -7 <= iy < 7 else None


def main():
    d = sys.argv[1]

    pts = defaultdict(list)
    with open(os.path.join(d, "trajectories.csv")) as f:
        for r in csv.DictReader(f):
            pts[r["robot_id"]].append((float(r["x"]), float(r["y"])))

    vis = set()
    for rb, ps in pts.items():
        for p in ps:
            c = cell(*p)
            if c:
                vis.add(c)
        for (x0, y0), (x1, y1) in zip(ps, ps[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            n = max(2, int(L / 0.02))
            for i in range(n + 1):
                t = i / n
                c = cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
                if c:
                    vis.add(c)
    cov = len(vis)

    with open(os.path.join(d, "coverage_final.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cell_x", "cell_y", "visited"])
        for (x, y) in GRID:
            w.writerow([x, y, 1 if (x, y) in vis else 0])

    p = os.path.join(d, "summary.csv")
    out = []
    for ln in open(p).read().splitlines():
        if ln.startswith("visited_cells,"):
            ln = "visited_cells,%d" % cov
        elif ln.startswith("coverage_pct,"):
            ln = "coverage_pct,%.2f" % (cov / 196.0 * 100.0)
        out.append(ln)
    open(p, "w").write("\n".join(out) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Final Global Coverage Map")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.set_xlim(-7, 7)
    ax.set_ylim(-7, 7)
    for (cx, cy) in GRID:
        color = "green" if (cx, cy) in vis else "red"
        ax.add_patch(plt.Rectangle((cx, cy), 1.0, 1.0,
                                   facecolor=color, edgecolor="black", alpha=0.5))
    for rid in sorted(pts, key=lambda r: int(r.split("_")[1])):
        xs = [p[0] for p in pts[rid]]
        ys = [p[1] for p in pts[rid]]
        ax.plot(xs, ys, label=rid)
    ax.legend(loc="upper right", fontsize="small")
    ax.text(0.05, 0.95, "Visited %d/196 cells" % cov,
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            bbox=dict(facecolor="white", alpha=0.6, edgecolor="none"))
    fig.savefig(os.path.join(d, "coverage_map.png"), dpi=150)
    plt.close(fig)

    print("patched %s -> visited %d/196 (%.2f%%)" % (d, cov, cov / 196.0 * 100.0))


if __name__ == "__main__":
    main()
