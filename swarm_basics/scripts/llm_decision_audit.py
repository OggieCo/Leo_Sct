#!/usr/bin/env python3
"""llm_decision_audit.py — tally the LLM decision layer's actions.

Reads the per-run decision logs of the BT+LLM configuration
(<root>/<scenario>/<batch_N>/llm_usage.csv, one row per decision) and reports:

  * action inventory + frequency (total, share, mean per run, min/max per run)
  * action x scenario matrix (mean count per run)
  * validity: rows whose action label is outside the declared action space

Usage:
    python3 scripts/llm_decision_audit.py [--root DIR] [--out-dir DIR]
                                         [--include-obstacles]
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

ACTIONS = ("proceed", "yield", "slow", "arc_left", "arc_right", "stop", "back")
DEFAULT_ROOT = "results/coverage_logs/BT_LLM"
DEFAULT_OUT = "results/analysis"


def scenario_key(scen: str):
    """Sort key: (rovers, humans) for the obstacle-free layouts."""
    parts = scen.split("_")
    try:
        return (int(parts[0]), int(parts[2]))
    except (IndexError, ValueError):
        return (99, 99)


def collect(root: str, include_obstacles: bool):
    per_run = defaultdict(list)      # scenario -> list of Counter(dict)
    unknown = defaultdict(int)       # bad label -> rows
    runs = defaultdict(int)
    for scen in sorted(os.listdir(root)):
        sdir = os.path.join(root, scen)
        if not os.path.isdir(sdir):
            continue
        if "obstacle" in scen and not include_obstacles:
            continue
        for batch in sorted(os.listdir(sdir)):
            f = os.path.join(sdir, batch, "llm_usage.csv")
            if not os.path.isfile(f):
                continue
            counts = defaultdict(int)
            with open(f, newline="") as fh:
                for row in csv.DictReader(fh):
                    act = (row.get("action") or "").strip()
                    if act not in ACTIONS:
                        unknown[act] += 1
                        continue
                    counts[act] += 1
            per_run[scen].append(dict(counts))
            runs[scen] += 1
    return per_run, runs, unknown


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--out-dir", default=DEFAULT_OUT)
    ap.add_argument("--include-obstacles", action="store_true")
    a = ap.parse_args()

    per_run, runs, unknown = collect(a.root, a.include_obstacles)
    scens = sorted(per_run, key=scenario_key)
    if not scens:
        print(f"no logs under {a.root}", file=sys.stderr)
        return 1
    n_runs_total = sum(runs[s] for s in scens)

    # ---- matches: rows per run (sanity) -------------------------------------
    tot_decisions = sum(sum(c.values()) for s in scens for c in per_run[s])
    print(f"scenarios: {len(scens)}   runs: {n_runs_total}   "
          f"decisions: {tot_decisions}   decisions/run: {tot_decisions / n_runs_total:.1f}")
    if unknown:
        print("UNKNOWN LABELS:", dict(unknown))
    else:
        print("validity: all rows carry a label from the declared action space")

    # ---- Table A: action x scenario (mean count per run) --------------------
    scen_means = {s: {act: mean([c.get(act, 0) for c in per_run[s]]) for act in ACTIONS}
                  for s in scens}
    hdr = "action".ljust(10) + "".join(f"{s.split('_')[0]}r{s.split('_')[2]}h".rjust(7) for s in scens) + "   mean/run"
    print("\nTable A - mean decisions per run (action x layout)")
    print(hdr)
    print("-" * len(hdr))
    for act in ACTIONS:
        row = act.ljust(10)
        vals = []
        for s in scens:
            v = scen_means[s][act]
            vals.append(v)
            row += f"{v:7.1f}"
        row += f"{mean(vals):11.1f}"
        print(row)
    # column totals
    row = "all".ljust(10)
    for s in scens:
        row += f"{mean([sum(c.values()) for c in per_run[s]]):7.1f}"
    row += f"{tot_decisions / n_runs_total:11.1f}"
    print(row)

    # ---- Table B: overall frequency ----------------------------------------
    all_counts = [c for s in scens for c in per_run[s]]
    total_by_act = {act: sum(c.get(act, 0) for c in all_counts) for act in ACTIONS}
    per_run_by_act = defaultdict(list)
    for c in all_counts:
        for act in ACTIONS:
            per_run_by_act[act].append(c.get(act, 0))

    hdr = f"{'action':10}{'total':>8}{'share %':>9}{'mean/run':>10}{'min':>6}{'max':>6}"
    print("\nTable B - action frequency over all runs")
    print(hdr)
    print("-" * len(hdr))
    for act in sorted(ACTIONS, key=lambda x: -total_by_act[x]):
        xs = per_run_by_act[act]
        print(f"{act:10}{total_by_act[act]:8d}"
              f"{100.0 * total_by_act[act] / tot_decisions:9.1f}"
              f"{mean(xs):10.1f}{min(xs):6d}{max(xs):6d}")
    print(f"{'all':10}{tot_decisions:8d}{100.0:9.1f}{tot_decisions / n_runs_total:10.1f}")

    # ---- write CSVs ---------------------------------------------------------
    os.makedirs(a.out_dir, exist_ok=True)
    out_a = os.path.join(a.out_dir, "llm_actions_by_layout.csv")
    with open(out_a, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["action"] + scens + ["mean_per_run"])
        for act in ACTIONS:
            row = [act] + [f"{scen_means[s][act]:.3f}" for s in scens]
            row.append(f"{mean([scen_means[s][act] for s in scens]):.3f}")
            w.writerow(row)
    out_b = os.path.join(a.out_dir, "llm_actions_overall.csv")
    with open(out_b, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["action", "total", "share_pct", "mean_per_run", "min_per_run", "max_per_run"])
        for act in sorted(ACTIONS, key=lambda x: -total_by_act[x]):
            xs = per_run_by_act[act]
            w.writerow([act, total_by_act[act],
                        f"{100.0 * total_by_act[act] / tot_decisions:.2f}",
                        f"{mean(xs):.3f}", min(xs), max(xs)])
    print(f"\nwritten: {out_a}\n         {out_b}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
