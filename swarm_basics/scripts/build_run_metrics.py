#!/usr/bin/env python3
"""build_run_metrics.py — tidy per-run metrics table for the results analysis.

One row per run folder under results/coverage_logs/<MODE>/<scenario>/batch_N.
Columns (Ecem's comparison set + context):
  - collisions:            bumps_total + breakdown wall/human/robot
                           (social_events.csv BUMP episodes, pair-deduplicated)
  - path lengths:          total_path_length_m (summary.csv)
  - social-distance        human_close / robot_close episode counts
    interruptions:         (0->1 transitions in social_state.csv, per run)
                           + HUMAN_SEEN / ROBOT_NEAR / OBSTACLE_NEAR counts
  - goals:                 dispatched / reached / goal_failures / scans (goal_stats.csv)
  - context:               duration_sec, coverage_pct
  - LLM (BT_LLM only):     llm_calls, llm_cost_usd

Usage (host or container):
    python3 swarm_basics/scripts/build_run_metrics.py [--root DIR] [--out FILE]

Default output: <repo>/results/analysis/run_metrics.csv
"""
import argparse
import csv
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

MODES = {"Nav2": "no", "BT": "reactive", "BT_LLM": "ai"}

FIELDS = [
    "mode", "arm", "scenario", "repeat",
    "duration_sec", "coverage_pct", "path_total_m",
    "bumps_total", "bumps_episodes", "bumps_wall", "bumps_human",
    "bumps_robot", "bumps_obstacle", "bumps_other",
    "human_close_episodes", "robot_close_episodes",
    "human_seen", "robot_near", "obstacle_near",
    "goals_sent", "goals_succeeded", "goal_failures", "goals_scan",
    "llm_calls", "llm_cost_usd",
    "run_dir",
]


def read_summary(path):
    """summary.csv is long format (metric,value) — return dict."""
    d = {}
    try:
        with open(path, newline="") as fh:
            for row in csv.reader(fh):
                if len(row) >= 2 and row[0].strip() not in ("", "metric"):
                    d[row[0].strip()] = row[1].strip()
    except OSError:
        pass
    return d


def read_goals(path):
    """goal_stats.csv: robot_id,goals_sent,goals_succeeded,goal_failures,scan_goals

    Read positionally, so it works for runs collected before 2026-10-14, whose
    header still says `goals_failed` (same quantity: failed attempts).
    """
    tot = [0, 0, 0, 0]
    try:
        with open(path, newline="") as fh:
            for i, row in enumerate(csv.reader(fh)):
                if i == 0 or len(row) < 5:
                    continue
                for j in range(4):
                    try:
                        tot[j] += int(float(row[1 + j]))
                    except ValueError:
                        pass
    except OSError:
        pass
    return tot  # sent, succeeded, failed, scan


def read_social_summary(path):
    c = Counter()
    try:
        with open(path, newline="") as fh:
            for i, row in enumerate(csv.reader(fh)):
                if i == 0 or len(row) < 3:
                    continue
                try:
                    c[row[1].strip()] += int(float(row[2]))
                except ValueError:
                    pass
    except OSError:
        pass
    return c


def read_close_episodes(path):
    """social_state.csv -> (human_close 0->1 count, robot_close 0->1 count).

    Large file: stream row by row; track previous state per robot.
    """
    hc = rc = 0
    prev = {}  # robot -> (human_close, robot_close)
    try:
        with open(path, newline="") as fh:
            rd = csv.DictReader(fh)
            for row in rd:
                rb = row.get("robot_id")
                h = row.get("human_close", "0").strip() == "1"
                r = row.get("robot_close", "0").strip() == "1"
                ph, pr = prev.get(rb, (False, False))
                if h and not ph:
                    hc += 1
                if r and not pr:
                    rc += 1
                prev[rb] = (h, r)
    except OSError:
        pass
    return hc, rc


def read_contacts(path):
    """social_events.csv -> Counter over BUMP partners.

    Source of truth = the curated BUMP episode stream: the bumper node emits
    ONE row per contact episode per rover and names the counterpart
    (`other=human_3_bumper`, `other=robot_4`, `other=obstacle_1`).  Raw
    CONTACT_BEGIN rows in events.csv are deliberately NOT used: one physical
    encounter can publish several of them (multiple bumper links / repeat
    triggers inside a cooldown), which produced phantom collisions
    (verified 2026-10-15).

    A rover-rover contact is logged by BOTH rovers, so reciprocal rows of the
    same pair within PAIR_WINDOW seconds are merged — one collision per
    unordered pair — which makes the breakdown additive.
    """
    PAIR_WINDOW = 1.0
    bumps = []
    try:
        with open(path, newline="") as fh:
            for row in csv.DictReader(fh):
                if row.get("event_type", "").strip() != "BUMP":
                    continue
                det = row.get("details", "")
                name = ""
                for tok in det.split():
                    if tok.startswith("other="):
                        name = tok.split("=", 1)[1]
                        break
                low = name.lower()
                if "wall" in low:
                    kind = "wall"
                elif "human" in low:
                    kind = "human"
                elif "robot" in low:
                    kind = "robot"
                elif "obstacle" in low or "box" in low or "cylinder" in low:
                    kind = "obstacle"
                else:
                    kind = "other"
                try:
                    t = float(row.get("elapsed_sec", "0"))
                except ValueError:
                    t = 0.0
                bumps.append((kind, t, row.get("robot_id", ""), name))
    except OSError:
        return Counter()

    c = Counter()
    used = [False] * len(bumps)
    for i, (kind, t, robot, partner) in enumerate(bumps):
        if used[i]:
            continue
        used[i] = True
        if kind == "robot":                    # merge the reciprocal row
            for j in range(i + 1, len(bumps)):
                k2, t2, r2, p2 = bumps[j]
                if used[j] or k2 != "robot" or abs(t2 - t) > PAIR_WINDOW:
                    continue
                if r2 == partner and p2 == robot:
                    used[j] = True
                    break
        c[kind] += 1
    return c


def read_llm(path):
    calls = 0
    cost = 0.0
    try:
        with open(path, newline="") as fh:
            for i, row in enumerate(csv.reader(fh)):
                if i == 0 or len(row) < 6:
                    continue
                try:
                    calls += int(float(row[1]))
                    cost += float(row[5])
                except ValueError:
                    pass
    except OSError:
        pass
    return calls, cost


def num(d, key, default="0"):
    try:
        return float(d.get(key, default) or default)
    except (TypeError, ValueError):
        return 0.0


def main():
    ap = argparse.ArgumentParser()
    here = Path(__file__).resolve()
    ap.add_argument("--root", default=str(here.parents[2] / "results" / "coverage_logs"),
                    help="coverage_logs root (default: <repo>/results/coverage_logs)")
    ap.add_argument("--out", default=None,
                    help="output CSV (default: <repo>/results/analysis/run_metrics.csv)")
    a = ap.parse_args()

    root = Path(a.root)
    out = Path(a.out) if a.out else root.parent / "analysis" / "run_metrics.csv"
    out.parent.mkdir(parents=True, exist_ok=True)

    rows = []
    warn = []
    for mode, arm in MODES.items():
        mroot = root / mode
        if not mroot.is_dir():
            continue
        for sdir in sorted(mroot.iterdir()):
            if not sdir.is_dir():
                continue
            # Exclude the legacy 1-rover scenarios (stale era, not part of
            # the thesis comparison matrix) — 2026-10-12.
            if sdir.name.startswith("1_rovers"):
                continue
            for rdir in sorted(sdir.iterdir()):
                if not rdir.is_dir() or not rdir.name.startswith("batch_"):
                    continue
                # batch_5 and batch_6_run_2026-10-12_15-49-32 both -> seed 5/6
                m = re.match(r"batch_(\d+)", rdir.name)
                rep = int(m.group(1)) if m else 0

                s = read_summary(rdir / "summary.csv")
                sent, ok, fail, scan = read_goals(rdir / "goal_stats.csv")
                ss = read_social_summary(rdir / "social_summary.csv")
                hc, rc = read_close_episodes(rdir / "social_state.csv")
                cc = read_contacts(rdir / "social_events.csv")
                calls, cost = read_llm(rdir / "llm_summary.csv")

                if not s:
                    # Incomplete run (still driving, aborted boot or quarantined
                    # folder): not part of the dataset — skip it so half-written
                    # cells cannot enter the aggregates as zeros (2026-10-13).
                    warn.append(f"{rdir}: no summary.csv")
                    continue

                rows.append({
                    "mode": mode, "arm": arm, "scenario": sdir.name,
                    "repeat": rep,
                    "duration_sec": num(s, "duration_sec"),
                    "coverage_pct": num(s, "coverage_pct"),
                    "path_total_m": num(s, "total_path_length_m"),
                    "bumps_episodes": int(num(s, "total_bumps")),
                    "bumps_wall": cc.get("wall", 0),
                    "bumps_human": cc.get("human", 0),
                    "bumps_robot": cc.get("robot", 0),
                    "bumps_obstacle": cc.get("obstacle", 0),
                    "bumps_other": cc.get("other", 0),
                    "bumps_total": sum(cc.values()),
                    "human_close_episodes": hc,
                    "robot_close_episodes": rc,
                    "human_seen": ss.get("HUMAN_SEEN", 0),
                    "robot_near": ss.get("ROBOT_NEAR", 0),
                    "obstacle_near": ss.get("OBSTACLE_NEAR", 0),
                    "goals_sent": sent, "goals_succeeded": ok,
                    "goal_failures": fail, "goals_scan": scan,
                    "llm_calls": calls, "llm_cost_usd": f"{cost:.6f}",
                    "run_dir": str(rdir),
                })

    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    for m in sorted(warn):
        print(f"WARN {m}", file=sys.stderr)
    per = Counter((r["mode"], r["scenario"]) for r in rows)
    bad = {k: v for k, v in per.items() if v != 5}
    print(f"wrote {out} — {len(rows)} runs")
    print(f"cells with != 5 runs: {bad if bad else 'none'}")


if __name__ == "__main__":
    main()
