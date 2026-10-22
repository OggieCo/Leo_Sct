#!/usr/bin/env python3
"""Exposure analysis for the collision metric.

Reproduces the numbers quoted in the Results chapter:

  "Pooled over all layouts without obstacles, the stock configuration rises from
   3.3 collisions per run with three agents present to 7.5, 9.9, 12.3, 15.3 and
   18.4 collisions per run with eight agents (r = 0.64; least-squares slope
   +2.8 collisions per run per additional agent, R^2 = 0.41, n = 120)."

Definitions used here
---------------------
agent        = one rover or one human sharing the arena; read from the scenario
               folder name ("5_rovers_3_humans" -> 5 + 3 = 8 agents).  Layouts
               whose name contains "obstacles" are excluded, because there the
               dominant collision partner is static geometry rather than an
               agent.
observation  = one run; x = agents present, y = collisions per run
               (run_metrics.csv column `bumps_total`).
pooled       = all runs of that arm across the non-obstacle layouts are placed
               in one sample (12 layouts x 10 runs = 120 runs for Nav2 and BT).
statistic    = Pearson correlation r, ordinary least-squares slope
               dy/dx = cov(x, y) / var(x), and R^2 = r^2 (both are computed
               here without scipy, from the population moments).

Usage:  python3 scripts/collision_exposure.py [--csv results/analysis/run_metrics.csv]
"""

from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve()
DEFAULT_CSV = HERE.parents[2] / "results" / "analysis" / "run_metrics.csv"
ARMS = [("no", "Nav2"), ("reactive", "BT"), ("ai", "BT+LLM")]


def layout_size(scenario: str):
    """"3_rovers_2_humans" -> (3, 2); None for obstacle layouts."""
    if "obstacles" in scenario:
        return None
    parts = scenario.split("_")
    try:
        return int(parts[0]), int(parts[2])
    except (IndexError, ValueError):
        return None


def moments(xs, ys):
    """-> (r, slope, r_squared) of an ordinary least-squares line y = a + b x."""
    n = len(xs)
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    r = sxy / (n * statistics.pstdev(xs) * statistics.pstdev(ys))
    slope = sxy / sxx
    return r, slope, r * r


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv, newline="")))
    print(f"source: {a.csv}")

    for arm, label in ARMS:
        xs, ys, by_agents = [], [], defaultdict(list)
        for r in rows:
            if r["arm"] != arm:
                continue
            size = layout_size(r["scenario"])
            if size is None:
                continue                      # obstacle layouts: excluded
            rovers, humans = size
            y = float(r["bumps_total"])
            xs.append(rovers + humans)
            ys.append(y)
            by_agents[rovers + humans].append(y)

        r, slope, r2 = moments(xs, ys)
        print(f"\n=== {label} ({arm}) — non-obstacle layouts, n = {len(ys)} runs")
        print("  agents  layouts  runs  mean collisions/run")
        for k in sorted(by_agents):
            v = by_agents[k]
            print(f"  {k:6d}  {len(v) // 10:7d}  {len(v):4d}  {statistics.mean(v):9.2f}")
        print(f"  Pearson r(agents, collisions) = {r:+.2f}")
        print(f"  least-squares slope           = {slope:+.2f} collisions per run "
              f"per additional agent")
        print(f"  R^2                           = {r2:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
