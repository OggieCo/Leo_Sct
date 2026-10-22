#!/usr/bin/env python3
"""verify_llm_execution.py — check that the LLM decision layer's output reaches
the actuators.

For every decision in <run>/llm_usage.csv (timestamp, robot_id, action) the
matching robot state samples of <run>/social_state.csv are inspected in a short
window after the decision.  Two independent signals are used:

  cmd_lin / cmd_ang     last *commanded* twist  (/{ns}/cmd_vel)
  actual_speed_kph      measured speed          (/{ns}/odom)
  speed_scale           social speed cap applied by velocity_adaptor
  backwards             negative commanded linear velocity flag

Expected signature per action (BT branch in config/bt/social_nav.xml):

  proceed   AiProceed   -> FollowPath                 : moving forward
  slow      AiSlow      -> FollowPath (scaled)        : speed_scale < 1
  yield     AiYield     -> Wait 1.0 s loop (<=10 s)   : actual speed ~ 0
  stop      AiStop      -> Wait 5 s                   : actual speed ~ 0
  arc_left  AiArcLeft   -> Arc direction="left"       : cmd_ang > 0 in window
  arc_right AiArcRight  -> Arc direction="right"      : cmd_ang < 0 in window
  back      AiBack      -> BackUp 0.6 m @ 0.3 m/s     : cmd_lin < 0 / backwards

Usage:
    python3 scripts/verify_llm_execution.py [--root DIR] [--window S]
                                            [--probe N] [--out FILE]
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

ACTIONS = ("proceed", "yield", "slow", "arc_left", "arc_right", "stop", "back")
DEFAULT_ROOT = "results/coverage_logs/BT_LLM"
DEFAULT_OUT = "results/analysis/llm_execution_check.csv"

LIN_STOP = 0.05          # m/s, "not commanding motion"
ANG_MOVE = 0.05          # rad/s, "turning"
SPEED_STOP = 0.2         # km/h, |measured speed| below this = not moving
MOVE_KPH = 0.3           # km/h, measured speed above this = moving
SCALE_SLOW = 0.95        # speed_scale below this = reduced speed
BACK_FRAC = 0.2          # fraction of window samples flagged as backwards


def load_state(path):
    """robot_id -> (ts[], cmd_lin[], cmd_ang[], kph[], scale[], back[])."""
    per = {}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            rid = row["robot_id"]
            s = per.setdefault(rid, ([], [], [], [], [], []))
            try:
                s[0].append(float(row["timestamp"]))
                s[1].append(float(row["cmd_lin"]))
                s[2].append(float(row["cmd_ang"]))
                s[3].append(float(row["actual_speed_kph"]))
                s[4].append(float(row["speed_scale"]))
                s[5].append(1.0 if row["backwards"].strip() in ("1", "True", "true") else 0.0)
            except (ValueError, KeyError):
                s[0].pop()
                for i in (1, 2, 3, 4, 5):
                    s[i].pop()
    return per


def load_decisions(path):
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            act = (row.get("action") or "").strip()
            if act not in ACTIONS:
                continue
            out.append((float(row["timestamp"]), row["robot_id"], act,
                        (row.get("reason") or "").strip()))
    return out


def window_stats(state, ts, w):
    """Mean/fraction statistics over the samples in [ts, ts + w]."""
    t, lin, ang, kph, sc, bk = state
    lo = ts
    hi = ts + w
    idx = [i for i, x in enumerate(t) if lo <= x <= hi]
    if not idx:
        return None
    n = len(idx)
    return {
        "n": n,
        "cmd_lin": sum(lin[i] for i in idx) / n,
        "cmd_ang": sum(ang[i] for i in idx) / n,
        "kph": sum(kph[i] for i in idx) / n,
        "scale": sum(sc[i] for i in idx) / n,
        "back_frac": sum(bk[i] for i in idx) / n,
        # abs(): reverse motion is logged as negative speed
        "still_frac": sum(1.0 for i in idx if abs(kph[i]) <= SPEED_STOP) / n,
        "min_lin": min(lin[i] for i in idx),
        "max_kph": max(abs(kph[i]) for i in idx),
    }


def verdict(action, st):
    """True when the window matches the branch the action selects."""
    if st is None:
        return None
    if action == "proceed_det":
        return st["cmd_lin"] > LIN_STOP or st["max_kph"] > MOVE_KPH
    if action == "proceed":
        # moving forward at some point in the window, by command or by odometry
        return st["cmd_lin"] > LIN_STOP or st["max_kph"] > MOVE_KPH
    if action == "slow":
        return st["scale"] < SCALE_SLOW
    if action in ("yield", "stop"):
        return st["still_frac"] >= 0.5
    if action == "arc_left":
        return st["cmd_ang"] > ANG_MOVE
    if action == "arc_right":
        return st["cmd_ang"] < -ANG_MOVE
    if action == "back":
        return st["min_lin"] < -LIN_STOP or st["back_frac"] >= BACK_FRAC
    return None


def runs(root):
    for scen in sorted(os.listdir(root)):
        sdir = os.path.join(root, scen)
        if not os.path.isdir(sdir) or "obstacle" in scen:
            continue
        for batch in sorted(os.listdir(sdir)):
            bdir = os.path.join(sdir, batch)
            d = os.path.join(bdir, "llm_usage.csv")
            s = os.path.join(bdir, "social_state.csv")
            if os.path.isfile(d) and os.path.isfile(s):
                yield scen, batch, bdir, d, s


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--window", type=float, default=2.0,
                    help="seconds after a decision inspected for its effect "
                         "(the state logger samples at roughly 1 Hz)")
    ap.add_argument("--probe", type=int, default=0,
                    help="print the first N decisions per action with their window stats")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--by-layout", default="2_rovers_1_humans,3_rovers_3_humans,5_rovers_3_humans",
                    help="comma-separated layouts for the per-layout breakdown")
    a = ap.parse_args()

    layouts = [s for s in a.by_layout.split(",") if s]
    lay = defaultdict(lambda: defaultdict(lambda: [0, 0, 0]))   # scen -> act -> [n, ok, nostate]
    agree = defaultdict(int)
    total = defaultdict(int)
    nostate = defaultdict(int)
    det_rows = defaultdict(int)
    sums = defaultdict(lambda: defaultdict(list))
    probed = defaultdict(int)

    for scen, batch, bdir, dpath, spath in runs(a.root):
        state = load_state(spath)
        for ts, rid, act, reason in load_decisions(dpath):
            st = window_stats(state.get(rid, ([], [], [], [], [], [])), ts, a.window)
            v = verdict(act, st)
            # "human has cleared - resuming (deterministic release)" rows are
            # published by the planner's own guardrail, not chosen by the model
            if act == "proceed" and "deterministic" in reason:
                act = "proceed_det"
                det_rows[act] += 1
            total[act] += 1
            if scen in layouts:
                lay[scen][act][0] += 1
            if st is None:
                nostate[act] += 1
                if scen in layouts:
                    lay[scen][act][2] += 1
                continue
            if v:
                agree[act] += 1
                if scen in layouts:
                    lay[scen][act][1] += 1
            for k in ("cmd_lin", "cmd_ang", "kph", "scale", "back_frac", "still_frac"):
                sums[act][k].append(st[k])
            if a.probe and probed[act] < a.probe and "deterministic" not in reason:
                probed[act] += 1
                print(f"PROBE {scen}/{batch} t={ts:.2f} {rid} {act:9s} "
                      f"n={st['n']:2d} lin={st['cmd_lin']:+.3f}(min {st['min_lin']:+.3f}) "
                      f"ang={st['cmd_ang']:+.3f} kph={st['kph']:5.2f}(max|.| {st['max_kph']:4.2f}) "
                      f"scale={st['scale']:.2f} still={st['still_frac']:.2f} ok={v}  | {reason[:50]}")

    def mean(xs):
        return sum(xs) / len(xs) if xs else float("nan")

    hdr = (f"{'action':16}{'n':>7}{'agree':>7}{'%':>7}{'nostate':>8}"
           f"{'cmd_lin':>9}{'cmd_ang':>9}{'kph':>7}{'scale':>7}{'still':>7}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for act in ACTIONS + ("proceed_det",):
        n, ok, ns = total[act], agree[act], nostate[act]
        pct = 100.0 * ok / (n - ns) if n > ns else float("nan")
        print(f"{act:16}{n:7d}{ok:7d}{pct:7.1f}{ns:8d}"
              f"{mean(sums[act]['cmd_lin']):9.3f}{mean(sums[act]['cmd_ang']):9.3f}"
              f"{mean(sums[act]['kph']):7.2f}{mean(sums[act]['scale']):7.2f}"
              f"{mean(sums[act]['still_frac']):7.2f}")

    # ---- per-layout breakdown -------------------------------------------------
    if layouts:
        print("\nTable C - decisions, executed and execution rate per layout")
        head = f"{'layout':22}{'action':16}{'n':>7}{'exec':>7}{'%':>7}"
        print(head)
        print("-" * len(head))
        for scen in layouts:
            for act in ACTIONS + ("proceed_det",):
                n, ok, ns = lay[scen][act]
                pct = 100.0 * ok / (n - ns) if n > ns else float("nan")
                print(f"{scen:22}{act:16}{n:7d}{ok:7d}{pct:7.1f}")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    if layouts:
        out_c = os.path.join(os.path.dirname(a.out), "llm_execution_by_layout.csv")
        with open(out_c, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["scenario", "action", "n_decisions", "executed", "executed_pct"])
            for scen in layouts:
                for act in ACTIONS + ("proceed_det",):
                    n, ok, ns = lay[scen][act]
                    pct = 100.0 * ok / (n - ns) if n > ns else float("nan")
                    w.writerow([scen, act, n, ok, f"{pct:.1f}"])
        print(f"written: {out_c}")
    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["action", "n_decisions", "agree", "agree_pct", "no_state_samples",
                    "mean_cmd_lin", "mean_cmd_ang",
                    "mean_speed_kph", "mean_speed_scale", "mean_still_frac"])
        for act in ACTIONS + ("proceed_det",):
            n, ok, ns = total[act], agree[act], nostate[act]
            pct = 100.0 * ok / (n - ns) if n > ns else float("nan")
            w.writerow([act, n, ok, f"{pct:.1f}", ns, det_rows[act],
                        f"{mean(sums[act]['cmd_lin']):.4f}",
                        f"{mean(sums[act]['cmd_ang']):.4f}",
                        f"{mean(sums[act]['kph']):.3f}",
                        f"{mean(sums[act]['scale']):.3f}",
                        f"{mean(sums[act]['still_frac']):.3f}"])
    print(f"\nwritten: {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
