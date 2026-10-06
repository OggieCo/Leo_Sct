"""Scenario-aware validation of the fixed tour waypoints (TEST-HARNESS tool).

The per-robot tours in fixed_goals.py were authored for the open arena.  A
waypoint that falls inside a static obstacle footprint (+ clearance) is
physically unreachable: the global planner cannot plan INTO a lethal /
inscribed costmap cell, and Nav2's goal checker (xy 0.25 m) can never be
satisfied — the rover burns the run in plan-abort / scan-retry loops.

Seen 2026-09-14 in 3_rovers_and_obstacles: goal #2 = (0, -2) sits INSIDE the
box at (0.23, -1.89); robot_1 never advanced past it for the whole 300 s run
(Nav2 and BT arms too: 3-4 failed attempts then endless scan-retry loops).

sanitize_tour() snaps such waypoints radially out of the obstacle keepout
ring.  Deterministic (no RNG) -> every arm (Nav2 / BT / BT+LLM) sees the
exact same sanitized tour, so the comparison stays fair.  This is experiment
INPUT validation, not navigation policy — all navigation decisions stay in
the behavior trees.  No-op when the active scenario has no static obstacles.

Pure stdlib (no ROS imports) so it can be dry-run offline:
    python3 -c "..." see tests/dry-run in the session notes.
"""

import json
import math
import os

# Keep goal cells fully outside the global costmap inflation ring so the
# planner treats them as ordinary free cells (nav2_generic.yaml:
# global inflation_radius 0.7, robot_radius 0.3, goal xy tolerance 0.25).
CLEARANCE = 0.70


def load_obstacles(path=None):
    """Static obstacles of the active scenario ([] when none / no scenario).

    `path` defaults to the SWARM_SCENARIO_JSON env var exported by
    start_experiment.sh (--scenario NAME|N).
    """
    path = path or os.environ.get('SWARM_SCENARIO_JSON', '')
    if not path or not os.path.isfile(path):
        return []
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return []
    return [o for o in (data.get('obstacles') or []) if isinstance(o, dict)]


def _box_geom(o):
    x, y = float(o['pose'][0]), float(o['pose'][1])
    hx, hy = float(o['size'][0]) / 2.0, float(o['size'][1]) / 2.0
    yaw = float(o['pose'][3]) if len(o['pose']) > 3 else 0.0
    return x, y, hx, hy, yaw


def _dist_to_footprint(x, y, o):
    """Exact Euclidean distance from (x, y) to the footprint outline."""
    if o.get('type') == 'box':
        cx, cy, hx, hy, yaw = _box_geom(o)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - cx, y - cy
        ox = max(abs(dx * c + dy * s) - hx, 0.0)   # beyond local x edge
        oy = max(abs(-dx * s + dy * c) - hy, 0.0)  # beyond local y edge
        return math.hypot(ox, oy)
    cx, cy = float(o['pose'][0]), float(o['pose'][1])
    return max(0.0, math.hypot(x - cx, y - cy) - float(o['radius']))


def _min_clearance(x, y, obstacles):
    """Distance to the nearest footprint (inf when there are none)."""
    return min((_dist_to_footprint(x, y, o) for o in obstacles),
               default=float('inf'))


def inside_keepout(x, y, o, margin=CLEARANCE):
    """Is (x, y) within `margin` of this obstacle's footprint?"""
    return _dist_to_footprint(x, y, o) < margin


def _push_out(x, y, o, margin):
    """Radially push (x, y) out of obstacle `o`'s keepout ring."""
    cx, cy = float(o['pose'][0]), float(o['pose'][1])
    dx, dy = x - cx, y - cy
    if dx == 0.0 and dy == 0.0:
        dx, dy = 1.0, 0.0             # ON the centre: fixed fallback direction
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    if o.get('type') == 'box':
        hx, hy = float(o['size'][0]) / 2.0, float(o['size'][1]) / 2.0
        yaw = float(o['pose'][3]) if len(o['pose']) > 3 else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        ulx = ux * c + uy * s         # ray direction in the box frame
        uly = -ux * s + uy * c
        cand = [h / abs(u) for h, u in ((hx, ulx), (hy, uly)) if u]
        t_b = min(cand) if cand else 0.0   # centre -> footprint edge along ray
    else:
        t_b = float(o['radius'])
    t = t_b + margin + 0.05           # +5 cm hair to clear the `<=` test
    return cx + t * ux, cy + t * uy


def sanitize_waypoint(x, y, obstacles, margin=CLEARANCE):
    """Snap one waypoint out of every obstacle keepout ring.

    Returns (x', y', pushed_from_or_None); pushed_from = the obstacle centre
    the point was pushed away from (for logging), None when already clear.
    Iterates nearest-obstacle pushes because a pushed point can land in a
    NEIGHBOURING obstacle's ring (box and cylinder can be ~1.9 m apart),
    and steps along the push ray to reach the EXACT clearance — a radial
    jump adds less than `margin` of perpendicular clearance when the exit
    is near a rectangle corner (would otherwise be a fixed point a few mm
    short of the ring, seen in the first dry-run 2026-09-14).
    """
    pushed_from = None
    px, py = float(x), float(y)
    for _ in range(12):               # converges in 2-4 passes in practice
        if _min_clearance(px, py, obstacles) >= margin + 0.005:
            break
        o = min(obstacles, key=lambda oo: _dist_to_footprint(px, py, oo))
        if pushed_from is None:
            pushed_from = (float(o['pose'][0]), float(o['pose'][1]))
        px, py = _push_out(px, py, o, margin)
        ux = px - float(o['pose'][0])
        uy = py - float(o['pose'][1])
        n = math.hypot(ux, uy)
        if n:
            ux, uy = ux / n, uy / n
            for _ in range(80):       # corner/oblique-ray haircut (<= 4 m)
                if not inside_keepout(px, py, o, margin):
                    break
                px += 0.05 * ux
                py += 0.05 * uy
    if pushed_from is None:
        return x, y, None
    return round(px, 2), round(py, 2), pushed_from


def sanitize_tour(goals, obstacles, margin=CLEARANCE):
    """Sanitize a whole waypoint list (deterministic, order-preserving).

    Returns (sanitized_list, changes) with
    changes = [(idx, old_xy, new_xy, pushed_from_xy), ...].
    """
    if not obstacles:
        return list(goals), []
    out, changes = [], []
    for i, (gx, gy) in enumerate(goals):
        nx, ny, pushed = sanitize_waypoint(gx, gy, obstacles, margin)
        if pushed is not None:
            changes.append((i, (gx, gy), (nx, ny), pushed))
        out.append((nx, ny))
    return out, changes
