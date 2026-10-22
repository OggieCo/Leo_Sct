#!/usr/bin/env python3
"""count_social_actions.py — count how often each social action was selected,
per robot and per run, across all BT+LLM runs.

Source of truth: results/coverage_logs/BT_LLM/<scenario>/<run>/llm_usage.csv
(every row = one decision; column `action` ∈ proceed|slow|stop|yield|back|
arc_left|arc_right; column `reason` distinguishes LLM-decided vs deterministic
releases, e.g. "human has cleared — resuming (deterministic release)").

Usage:
    python3 scripts/count_social_actions.py [--root results/coverage_logs]
                                            [--out counts.csv]
                                            [--split-deterministic]

Outputs (stdout unless --out):
  1) per-run-per-robot action counts
  2) scenario-level totals (summed over robots and runs)
"""
import argparse
import csv
import os
import sys
from collections import Counter, defaultdict

ACTIONS = ["proceed", "slow", "stop", "yield", "back", "arc_left", "arc_right"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results/coverage_logs")
    ap.add_argument("--out", default=None, help="write per-robot counts to this CSV")
    ap.add_argument("--split-deterministic", action="store_true",
                    help="add *_det columns for rows whose reason contains 'deterministic'")
    a = ap.parse_args()

    per_robot = []                                     # rows for output
    per_scen = defaultdict(Counter)                    # scenario -> action counts

    base = os.path.join(a.root, "BT_LLM")
    for scen in sorted(os.listdir(base)):
        sdir = os.path.join(base, scen)
        if not os.path.isdir(sdir):
            continue
        for run in sorted(os.listdir(sdir)):
            f = os.path.join(sdir, run, "llm_usage.csv")
            if not os.path.isfile(f):
                continue
            cnt = defaultdict(Counter)                 # robot -> action counts
            det = defaultdict(Counter)                 # robot -> deterministic counts
            with open(f) as fh:
                for row in csv.DictReader(fh):
                    rb, act = row["robot_id"], row["action"].strip()
                    cnt[rb][act] += 1
                    if "deterministic" in row["reason"]:
                        det[rb][act] += 1
            for rb in sorted(cnt, key=lambda r: int(r.split("_")[1])):
                row = {"scenario": scen, "run": run, "robot": rb}
                for act in ACTIONS:
                    row[act] = cnt[rb][act]
                    if a.split_deterministic:
                        row[act + "_det"] = det[rb][act]
                row["total"] = sum(cnt[rb][act] for act in ACTIONS)
                per_robot.append(row)
                per_scen[scen].update(cnt[rb])

    fields = ["scenario", "run", "robot"] + ACTIONS + \
             ([x + "_det" for x in ACTIONS] if a.split_deterministic else []) + ["total"]
    out = open(a.out, "w", newline="") if a.out else sys.stdout
    w = csv.DictWriter(out, fieldnames=fields)
    w.writeheader()
    w.writerows(per_robot)

    if a.out:
        out.close()
        print("wrote %d rows to %s" % (len(per_robot), a.out), file=sys.stderr)
        print("\nScenario totals (robots+runs summed):", file=sys.stderr)
        for scen in sorted(per_scen):
            c = per_scen[scen]
            print("  %-24s %s" % (scen, "  ".join("%s=%d" % (x, c[x]) for x in ACTIONS)),
                  file=sys.stderr)


if __name__ == "__main__":
    main()
