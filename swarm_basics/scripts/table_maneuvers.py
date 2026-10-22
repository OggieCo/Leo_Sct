#!/usr/bin/env python3
"""table_maneuvers.py — per-robot executed-maneuver table for one scenario.

Counts the START events of the four social maneuvers logged by the shared BT
condition nodes (identical in the BT and BT+LLM arms):

    yield   HUMAN_YIELD_START   + ROBOT_YIELD_START
    slow    HUMAN_SUPPRESS_START + ROBOT_SUPPRESS_START
    arc     ROBOT_AVOID_START
    backup  BACKWARD_START      (negative commanded velocity, any source)

Durations come from matched START/END pairs of the same robot and family.

Usage:
    python3 scripts/table_maneuvers.py --scenario 5_rovers_3_humans [--roots BT BT_LLM]
"""
import argparse
import csv
import os
from collections import defaultdict

FAM_START = {
    "yield":  {"HUMAN_YIELD_START", "ROBOT_YIELD_START"},
    "slow":   {"HUMAN_SUPPRESS_START", "ROBOT_SUPPRESS_START"},
    "arc":    {"ROBOT_AVOID_START"},
    "backup": {"BACKWARD_START"},
}
FAM_END = {
    "yield":  {"HUMAN_YIELD_END", "ROBOT_YIELD_END"},
    "slow":   {"HUMAN_SUPPRESS_END", "ROBOT_SUPPRESS_END"},
    "arc":    {"ROBOT_AVOID_END"},
    "backup": {"BACKWARD_END"},
}
FAMS = ["yield", "slow", "arc", "backup"]

# raw, un-merged action events (what --discrete reports)
DISCRETE = ["HUMAN_YIELD_START", "ROBOT_YIELD_START",
            "HUMAN_SUPPRESS_START", "ROBOT_SUPPRESS_START",
            "ROBOT_AVOID_START", "ROBOT_AVOID_END_ABORT",
            "BACKWARD_START"]


def scan(path, counts, durs, starts, raw=None):
    """starts: {name -> [t, ...]} — one list per (robot, exact event name).

    Each *_START pairs with its own *_END (`ROBOT_YIELD_END` closes
    `ROBOT_YIELD_START`, not any other yield); `starts` is reset per run by the
    caller, otherwise elapsed_sec wrapping between runs yields negative
    durations.
    """
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) < 5 or row[0] == "timestamp":
                continue
            try:
                t = float(row[1])
            except ValueError:
                continue
            robot = row[2]
            name = row[4] if row[3] == "BT" else row[3]
            key = (robot, name)
            if name in DISCRETE and raw is not None:
                raw[(robot, name)] += 1
            if name.endswith("_START"):
                starts[key].append(t)
                for fam in FAMS:
                    if name in FAM_START[fam]:
                        counts[(robot, fam)] += 1
                        break
            elif name.endswith("_END") and not name.endswith("_END_ABORT"):
                start_name = name[:-3] + "START"
                pend = starts[(robot, start_name)]
                if pend:
                    dt = t - pend.pop(0)
                    for fam in FAMS:
                        if name in FAM_END[fam]:
                            durs[(robot, fam)].append(dt)
                            break


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(here, "..", "..", "results",
                                                   "coverage_logs"))
    ap.add_argument("--scenario", default="5_rovers_3_humans")
    ap.add_argument("--arms", nargs="*", default=["BT", "BT_LLM"])
    ap.add_argument("--no-duration", action="store_true",
                    help="counts only, drop the mean episode length columns")
    ap.add_argument("--total", action="store_true",
                    help="one fleet-total row per arm instead of per-robot rows")
    ap.add_argument("--discrete", action="store_true",
                    help="raw action events, human/robot variants not merged")
    a = ap.parse_args()
    root = os.path.abspath(a.root)

    for arm in a.arms:
        counts, durs, raw = defaultdict(int), defaultdict(list), defaultdict(int)
        n_runs = 0
        sdir = os.path.join(root, arm, a.scenario)
        for run in sorted(os.listdir(sdir)):
            f = os.path.join(sdir, run, "social_events.csv")
            if os.path.isfile(f):
                # fresh pairing state per run (elapsed_sec restarts at 0)
                scan(f, counts, durs, defaultdict(list), raw)
                n_runs += 1
        if a.discrete:
            tot = [sum(v for (r, n), v in raw.items() if n == name)
                   for name in DISCRETE]
            print(f"| {arm} | " + " | ".join(str(x) for x in tot) + " |")
            continue
        robots = sorted({r for (r, _) in counts})
        if a.total:
            tot = [sum(counts[(r, fam)] for r in robots) for fam in FAMS]
            print(f"| {arm} | " + " | ".join(str(x) for x in tot) + " |")
            continue
        print(f"\n{arm} — {a.scenario}, {n_runs} runs; counts summed over runs"
              + ("" if a.no_duration else ", duration = mean episode length [s]"))
        if a.no_duration:
            print("| robot | " + " | ".join(FAMS) + " |")
            print("|---" * (len(FAMS) + 1) + "|")
            for r in robots:
                print(f"| {r} | " +
                      " | ".join(str(counts[(r, fam)]) for fam in FAMS) + " |")
            continue
        print("| robot | " + " | ".join(f"{f} | {f} [s]" for f in FAMS) + " |")
        print("|---" * (2 * len(FAMS) + 1) + "|")
        for r in robots:
            cells = []
            for fam in FAMS:
                d = durs[(r, fam)]
                cells.append(f"{counts[(r, fam)]} | "
                             f"{sum(d) / len(d):.1f}" if d else
                             f"{counts[(r, fam)]} | -")
            print(f"| {r} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
