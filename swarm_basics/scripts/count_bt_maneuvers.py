#!/usr/bin/env python3
"""count_bt_maneuvers.py — count executed social maneuvers per run, for the two
social arms (BT and BT_LLM), from each run's social_events.csv.

Both arms publish the SAME event vocabulary from the same BT condition nodes
(IsHumanClose / IsRobotClose), so the counts are directly comparable:
  yield     HUMAN_YIELD_START  + ROBOT_YIELD_START
  slow      HUMAN_SUPPRESS_START + ROBOT_SUPPRESS_START
  arc       ROBOT_AVOID_START
  arc_abort ROBOT_AVOID_END_ABORT
  reverse   BACKWARD_START                (separate event type, both arms)

For each family the mean duration is reported too, computed only from matched
START/END pairs per robot (an unfinished maneuver at the 300 s cutoff is counted
as a start with no end, which is why counts and durations use different n).

Usage:  python3 scripts/count_bt_maneuvers.py [--root DIR] [--out FILE]
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

ARMS = ["BT", "BT_LLM"]
FAM_START = {
    "yield": {"HUMAN_YIELD_START", "ROBOT_YIELD_START"},
    "slow": {"HUMAN_SUPPRESS_START", "ROBOT_SUPPRESS_START"},
    "arc": {"ROBOT_AVOID_START"},
    "arc_abort": {"ROBOT_AVOID_END_ABORT"},
    "reverse": {"BACKWARD_START"},          # event_type, not a BT detail
}
FAM_END = {
    "yield": {"HUMAN_YIELD_END", "ROBOT_YIELD_END"},
    "slow": {"HUMAN_SUPPRESS_END", "ROBOT_SUPPRESS_END"},
    "arc": {"ROBOT_AVOID_END"},
    "reverse": {"BACKWARD_END"},
}


def run_counts(path):
    """-> (counts per family, durations per family, run duration) for one run."""
    starts = defaultdict(list)      # (robot, family) -> [t, ...]
    counts = defaultdict(int)
    durs = defaultdict(list)
    tmax = 0.0
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) < 5 or row[0] == "timestamp":
                continue
            try:
                t = float(row[1])
            except ValueError:
                continue
            tmax = max(tmax, t)
            etype, detail = row[3], row[4]
            name = detail if etype == "BT" else etype
            for fam, members in FAM_START.items():
                if name in members:
                    counts[fam] += 1
                    starts[(row[2], fam)].append(t)
            for fam, members in FAM_END.items():
                if name in members:
                    key = (row[2], fam)
                    if starts[key]:
                        durs[fam].append(t - starts[key].pop(0))
    return counts, durs, tmax


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--root", default=os.path.join(here, "..", "..", "results",
                                                   "coverage_logs"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    root = os.path.abspath(a.root)

    agg = defaultdict(list)                          # (arm, scen, fam) -> [counts]
    dur = defaultdict(list)                          # (arm, scen, fam) -> [durations]
    fams = list(FAM_START)

    for arm in ARMS:
        for scen in sorted(os.listdir(os.path.join(root, arm))):
            sdir = os.path.join(root, arm, scen)
            if not os.path.isdir(sdir):
                continue
            for run in sorted(os.listdir(sdir)):
                f = os.path.join(sdir, run, "social_events.csv")
                if not os.path.isfile(f):
                    continue
                c, d, tmax = run_counts(f)
                if tmax <= 0:
                    continue
                for fam in fams:
                    agg[(arm, scen, fam)].append(c.get(fam, 0))
                    dur[(arm, scen, fam)].extend(d.get(fam, []))

    def mean(xs):
        return sum(xs) / len(xs) if xs else 0.0

    out_rows = []
    for (arm, scen, fam) in sorted(agg):
        xs = agg[(arm, scen, fam)]
        ds = dur[(arm, scen, fam)]
        out_rows.append([arm, scen, fam, len(xs), round(mean(xs), 2),
                         round(mean(ds), 1) if ds else ""])

    scen_order = ["2_rovers_1_humans", "3_rovers_3_humans", "5_rovers_3_humans"]
    print(f"{'arm':7} {'scenario':22} {'yield':>7} {'slow':>7} {'arc':>6} "
          f"{'abort':>6} {'rev':>6}   (mean count per ROVER per run)")
    for scen in scen_order:
        nrob = int(scen.split("_")[0])          # layouts differ in fleet size
        for arm in ARMS:
            cells = []
            for fam in ["yield", "slow", "arc", "arc_abort", "reverse"]:
                xs = agg.get((arm, scen, fam), [])
                cells.append(f"{mean(xs) / nrob:6.2f}" if xs else "     -")
            n = len(agg.get((arm, scen, "yield"), []))
            print(f"{arm:7} {scen:22} {' '.join(cells)}   n={n} rovers={nrob}")

    if a.out:
        with open(a.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["arm", "scenario", "family", "n_runs", "mean_count",
                        "mean_duration_s"])
            w.writerows(out_rows)
        print(f"\nwrote {a.out} ({len(out_rows)} rows)")


if __name__ == "__main__":
    sys.exit(main())
