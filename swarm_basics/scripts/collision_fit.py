#!/usr/bin/env python3
"""collision_fit.py — exposure fits for the collision metric (all 14 layouts).

Prints, per configuration:

  * the linear fit of collisions per run against the number of agents present,
    computed twice: over the twelve layouts without static obstacles and over all
    fourteen layouts;
  * a fit on all fourteen layouts that adds an indicator for the obstacle
    layouts:   collisions = a + b * agents + c * obstacle
    so the geometry effect is modelled instead of silently excluded;
  * the mean values of the two obstacle layouts, split into the contact types.

Rationale for the indicator: in the obstacle layouts the dominant collision
partner is static geometry rather than another agent, so a single agent-count
slope fitted over all layouts is biased downwards (Nav2: +2.82 -> +2.39).
Note that both obstacle layouts also have zero humans, so `c` mixes the effect of
the obstacles with the absence of pedestrians; with two such layouts the two
cannot be separated.

  python3 scripts/collision_fit.py [--csv results/analysis/run_metrics.csv]
"""

from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve()
DEFAULT_CSV = HERE.parents[2] / "results" / "analysis" / "run_metrics.csv"
ARMS = (("no", "Nav2"), ("reactive", "BT"), ("ai", "BT+LLM"))


def split(scenario: str):
    """-> (rovers, humans, is_obstacle_layout)"""
    p = scenario.split("_")
    if p[2] == "and":                    # "<R>_rovers_and_obstacles"
        return int(p[0]), 0, 1
    return int(p[0]), int(p[2]), 0


def pearson(xs, ys):
    mx, my = statistics.mean(xs), statistics.mean(ys)
    return (sum((a - mx) * (b - my) for a, b in zip(xs, ys))
            / (len(xs) * statistics.pstdev(xs) * statistics.pstdev(ys)))


def slope(xs, ys):
    mx, my = statistics.mean(xs), statistics.mean(ys)
    return (sum((a - mx) * (b - my) for a, b in zip(xs, ys))
            / sum((a - mx) ** 2 for a in xs))


def lstsq(X, y):
    """Least squares for a small design matrix X (list of rows)."""
    n = len(X[0])
    A = [[sum(X[i][a] * X[i][b] for i in range(len(X))) for b in range(n)]
         for a in range(n)]
    rhs = [sum(X[i][a] * y[i] for i in range(len(X))) for a in range(n)]
    M = [A[i][:] + [rhs[i]] for i in range(n)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[piv] = M[piv], M[c]
        for r in range(n):
            if r != c and M[r][c]:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv, newline="")))

    for arm, lab in ARMS:
        sub = [r for r in rows if r["arm"] == arm]
        for tag, sel in (("no-obstacle layouts", [r for r in sub
                                                  if "obstacles" not in r["scenario"]]),
                         ("all 14 layouts", sub)):
            xs = [sum(split(r["scenario"])[:2]) for r in sel]
            ys = [float(r["bumps_total"]) for r in sel]
            r_ = pearson(xs, ys)
            print(f"{lab:7s} {tag:20s} n={len(ys):3d}  mean={statistics.mean(ys):5.2f}"
                  f"  r={r_:+.2f}  slope={slope(xs, ys):+.2f}  R2={r_ * r_:.2f}")
        X = [[1.0, sum(split(r["scenario"])[:2]), split(r["scenario"])[2]] for r in sub]
        y = [float(r["bumps_total"]) for r in sub]
        c0, b1, c2 = lstsq(X, y)
        pred = [c0 + b1 * x[1] + c2 * x[2] for x in X]
        sst = sum((v - statistics.mean(y)) ** 2 for v in y)
        ssr = sum((v - p) ** 2 for v, p in zip(y, pred))
        print(f"        all 14 + obstacle indicator: {c0:+.2f} + {b1:+.2f}*agents "
              f"{c2:+.2f}*obstacle   (R2={1 - ssr / sst:.2f})\n")

    print("obstacle layouts, mean per run (total, human / rover):")
    d = defaultdict(list)
    for r in rows:
        if "obstacles" in r["scenario"]:
            d[(r["scenario"], r["arm"])].append(r)
    for (scen, arm), rs in sorted(d.items()):
        lab = dict(ARMS)[arm]
        m = lambda c: statistics.mean(float(r[c]) for r in rs)      # noqa: E731
        print(f"  {scen:24s} {lab:7s} n={len(rs):2d}  {m('bumps_total'):5.2f} "
              f"({m('bumps_human'):4.2f} / {m('bumps_robot'):4.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
