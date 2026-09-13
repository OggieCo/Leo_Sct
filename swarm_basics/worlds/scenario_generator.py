#!/usr/bin/env python3
"""Scenario generator for the scenario-based experiment sweep.

One SCENARIO = one reproducible setup: N rovers (spawn poses), M scripted
humans (walk lanes) and optionally static obstacles (boxes/cylinders).
All three behavior modes (Nav2 / BT / BT_LLM) are later run on the SAME
scenario, so the only difference between conditions is the behavior layer.

Usage (inside the container):
    python3 worlds/scenario_generator.py --scenario 3_rovers_3_humans
    python3 worlds/scenario_generator.py --scenario 9        # numbers also work

Writes (next to this file):
    worlds/scenarios/<name>/scenario.sdf     full Gazebo world
    worlds/scenarios/<name>/scenario.json    robots / humans / obstacles spec
with <name> from the SCENARIOS table, e.g. worlds/scenarios/2_rovers_2_humans/
(the name is also used for the run folders: <MODE>/<name>/run_<ts>/)

The SDF keeps <world name="human_world">, the same plugins (physics, sensors,
BumpableHuman, contact, scene broadcaster) and the same arena walls as
human_world.sdf, so every bridge topic / contact sensor / bumper stays
identical — only the spawns, humans and obstacles differ.
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../swarm_basics/worlds
OUT_ROOT = HERE / 'scenarios'

WORLD_NAME = 'human_world'                      # must match robot_config.WORLD_NAME

# Walk speed check: humans cross a <= 7 m lane in 14 s (0.5 m/s max).
LANE_MAX_SPAN = 7.0

# Minimum clearance kept around obstacles while placing them (metres).
MIN_OBSTACLE_GAP = 1.2

# ---------------------------------------------------------------------------
# Spawn layouts (DELIBERATE, not random): rovers start in the middle or in
# distinct corners, facing the arena centre.
# ---------------------------------------------------------------------------
SPAWNS_BY_ROVERS = {
    1: [(0.0, 0.0)],
    2: [(-4.5, -4.5), (4.5, 4.5)],                    # SW + NE corners
    3: [(-4.5, -4.5), (4.5, -4.5), (-4.5, 4.5)],      # three corners
}

# ---------------------------------------------------------------------------
# Human lanes (DELIBERATE): straight back-and-forth shuttles that never
# overlap each other, any rover spawn or the obstacles.  Two vertical lanes
# at x = +/-3.5 flank the rovers; the third human walks a short north
# shuttle between them (no lane ever passes through a spawn point).
#
# CRITICAL: lane ENDPOINTS must keep >= ~1 m clearance from the
# corner->centre diagonals — these are the rovers' first "scan 3 m ahead"
# paths:
#   robot_0 SW: line y =  x  -> old endpoint (-3.5,-3.5) sat ON it
#   robot_1 SE: line y = -x  -> old endpoint ( 3.5,-3.5) sat ON it
#   robot_2 NW: line y = -x  -> old endpoint (-3.5, 3.5) sat ON it
# A human standing on a corridor gets mapped as an obstacle blob on the
# rover's start cell -> NavFn fails every replan ("failed to create plan"),
# the recovery fallback loops and never heals (runs 2026-09-13 10:56/11:18).
# y = +/-2.0 gives ~1.06 m perpendicular clearance (0.0 m with +/-3.5);
# the north shuttle is trimmed to x = +/-1.5 for the same margin.
# ---------------------------------------------------------------------------
LANES_BY_HUMANS = {
    1: [[(-3.5, -2.0), (-3.5, 2.0)]],
    2: [[(-3.5, -2.0), (-3.5, 2.0)],
        [(3.5, -2.0), (3.5, 2.0)]],
    3: [[(-3.5, -2.0), (-3.5, 2.0)],
        [(3.5, -2.0), (3.5, 2.0)],
        [(-1.5, 3.0), (1.5, 3.0)]],
}

# ---------------------------------------------------------------------------
# SCENARIO SPEC — one entry per scenario.
#   name      : scenario name -> folders worlds/scenarios/<name>/ and
#               results/coverage_logs/<MODE>/<name>/
#   robots    : number of rovers (robot_0 .. robot_N-1); `spawns` = where
#   humans    : number of scripted actors; `lanes` = where they walk
#   boxes / cylinders : static obstacles (placed only when > 0)
#
# Scenarios 1..9 are the full 3x3 matrix of {1,2,3} rovers x {1,2,3} humans
# (row-major: 1r1h, 1r2h, 1r3h, 2r1h, 2r2h, 2r3h, 3r1h, 3r2h, 3r3h);
# scenario 10 adds the obstacle variation (boxes + cylinders) on the
# 3-rover / 3-human layout.  The numbers are short aliases only — the NAME
# is the scenario identifier used in commands and folder names.
# ---------------------------------------------------------------------------
SCENARIOS = {}
_scenario_no = 0
for _rovers in (1, 2, 3):
    for _humans in (1, 2, 3):
        _scenario_no += 1
        SCENARIOS[_scenario_no] = dict(
            name=f'{_rovers}_rovers_{_humans}_humans',
            robots=_rovers, humans=_humans, boxes=0, cylinders=0,
            spawns=SPAWNS_BY_ROVERS[_rovers],
            lanes=LANES_BY_HUMANS[_humans],
        )
SCENARIOS[_scenario_no + 1] = dict(
    name='3_rovers_and_obstacles',
    robots=3, humans=3, boxes=2, cylinders=2,
    spawns=SPAWNS_BY_ROVERS[3],
    lanes=LANES_BY_HUMANS[3],
)


def seg_point_dist(px, py, ax, ay, bx, by):
    """Shortest distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def build_humans(spec):
    """Human walk lanes from the SEEDS spec: one straight back-and-forth
    shuttle per human with staggered start delays.  The lanes are placed
    deliberately so humans never overlap each other, rovers or obstacles."""
    humans = []
    for i, (a, b) in enumerate(spec['lanes']):
        span = math.hypot(b[0] - a[0], b[1] - a[1])
        if span > LANE_MAX_SPAN:
            sys.exit(f'ERROR: {spec["name"]} lane {i} is {span:.1f} m long '
                     f'(> {LANE_MAX_SPAN} m) — walk speed would exceed 0.5 m/s')
        humans.append(dict(
            name=f'human_{i + 1}',
            waypoints=[[float(a[0]), float(a[1])],
                       [float(b[0]), float(b[1])]],
            delay=1.0 + 5.0 * i,
        ))
    return humans


def build_robots(spec):
    """Rover spawns from the SEEDS spec; every rover faces the arena centre
    (odom/map frames are anchored at the spawn pose, so any yaw is valid and
    a centre-facing yaw gives sensible initial odometry)."""
    robots = []
    for i, (x, y) in enumerate(spec['spawns']):
        # Corner spawns face the arena centre; a centre spawn faces +x
        # (atan2(-0.0, -0.0) would otherwise give -pi).
        yaw = 0.0 if (x == 0.0 and y == 0.0) else math.atan2(-y, -x)
        robots.append([f'robot_{i}', round(x, 3), round(y, 3), round(yaw, 4)])
    return robots


def build_obstacles(rng, n_boxes, n_cyl, robots, humans):
    """Static box/cylinder obstacles, clear of spawns and human lanes."""
    occupied = [(r[1], r[2]) for r in robots]
    obstacles = []

    def too_close(x, y, radius):
        for ox, oy in occupied:
            if math.hypot(x - ox, y - oy) < 2.0 * radius + MIN_OBSTACLE_GAP:
                return True
        for h in humans:
            ax, ay = h['waypoints'][0]
            bx, by = h['waypoints'][1]
            if seg_point_dist(x, y, ax, ay, bx, by) < radius + 0.9:
                return True
        return False

    for _ in range(n_boxes):
        for _ in range(1000):
            sx = rng.uniform(0.5, 1.2)
            sy = rng.uniform(0.5, 1.2)
            sz = rng.uniform(0.5, 1.2)
            radius = 0.5 * math.hypot(sx, sy)
            x = rng.uniform(-5.5, 5.5)
            y = rng.uniform(-5.5, 5.5)
            if too_close(x, y, radius):
                continue
            yaw = rng.uniform(0.0, math.pi)
            occupied.append((x, y))
            obstacles.append(dict(type='box',
                                  pose=[round(x, 3), round(y, 3), round(sz / 2, 3), round(yaw, 4)],
                                  size=[round(sx, 3), round(sy, 3), round(sz, 3)]))
            break

    for _ in range(n_cyl):
        for _ in range(1000):
            radius = rng.uniform(0.3, 0.6)
            length = rng.uniform(0.5, 1.5)
            x = rng.uniform(-5.5, 5.5)
            y = rng.uniform(-5.5, 5.5)
            if too_close(x, y, radius):
                continue
            occupied.append((x, y))
            obstacles.append(dict(type='cylinder',
                                  pose=[round(x, 3), round(y, 3), round(length / 2, 3)],
                                  radius=round(radius, 3),
                                  length=round(length, 3)))
            break

    return obstacles


# ---------------------------------------------------------------------------
# SDF emission — same skeleton/plugins/walls as human_world.sdf
# ---------------------------------------------------------------------------
def emit_sdf(seed, robots, humans, obstacles):
    pairs = '\n'.join(
        f'      <actor name="{h["name"]}" model="{h["name"]}_bumper"/>'
        for h in humans)

    parts = []
    parts.append(f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>200</real_time_update_rate>
    </physics>

    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
      <include_entity_names>true</include_entity_names>
    </plugin>
    <plugin
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands"> <!-- do no delete -->
    </plugin>

    <!-- Bumpable humans (same plugin as human_world.sdf): each scripted
         actor gets an invisible static bumper cylinder glued to it every
         step, so rovers physically bump against the human. -->
    <plugin
      filename="libleo_gz_bumpable_human_plugin.so"
      name="leo_gz::BumpableHuman">
{pairs}
    </plugin>
    <plugin
      filename="libignition-gazebo-contact-system.so"
      name="ignition::gazebo::systems::Contact">
      <topic>/world/{WORLD_NAME}/physics/contacts</topic>
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <light name='sun' type='directional'>
      <pose>0 0 10 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.5 -1</direction>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>10</range>
        <linear>1</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
    </light>

    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <surface><friction><ode/></friction></surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
''')

    # ---- walls (same as human_world.sdf) ----
    walls = [
        ('north_wall', 0, 7.25, 20, 0.5),
        ('south_wall', 0, -7.25, 20, 0.5),
        ('east_wall', 7.25, 0, 0.5, 20),
        ('west_wall', -7.25, 0, 0.5, 20),
    ]
    for name, x, y, size_x, size_y in walls:
        parts.append(f'''
    <model name="{name}">
      <pose>{x} {y} 1.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{size_x} {size_y} 2.0</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{size_x} {size_y} 2.0</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
      <static>true</static>
    </model>
''')

    # ---- obstacles ----
    for i, ob in enumerate(obstacles):
        x, y, z = ob['pose'][0], ob['pose'][1], ob['pose'][2]
        if ob['type'] == 'box':
            sx, sy, sz = ob['size']
            yaw = ob['pose'][3]
            parts.append(f'''
    <model name="obstacle_box_{i}">
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{sx} {sy} {sz}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx} {sy} {sz}</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      <static>true</static>
    </model>
''')
        else:
            radius, length = ob['radius'], ob['length']
            parts.append(f'''
    <model name="obstacle_cyl_{i}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.5 0.9 1</ambient>
            <diffuse>0.3 0.5 0.9 1</diffuse>
          </material>
        </visual>
      </link>
      <static>true</static>
    </model>
''')

    # ---- human actors + bumper cylinders ----
    for h in humans:
        (ax, ay), (bx, by) = h['waypoints']
        parts.append(f'''
    <actor name="{h['name']}">
      <skin>
        <filename>/usr/share/gazebo-11/media/models/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>/usr/share/gazebo-11/media/models/walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>{h['delay']:.1f}</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="LINEAR">
          <waypoint><time>0</time><pose>{ax} {ay} 1.0 0 0 0</pose></waypoint>
          <waypoint><time>5</time><pose>{ax} {ay} 1.0 0 0 0</pose></waypoint>
          <waypoint><time>19</time><pose>{bx} {by} 1.0 0 0 0</pose></waypoint>
          <waypoint><time>24</time><pose>{bx} {by} 1.0 0 0 0</pose></waypoint>
          <waypoint><time>38</time><pose>{ax} {ay} 1.0 0 0 0</pose></waypoint>
          <waypoint><time>43</time><pose>{ax} {ay} 1.0 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>

    <model name="{h['name']}_bumper">
      <static>true</static>
      <pose>0 0 100 0 0 0</pose>
      <link name="bumper_link">
        <pose>0 0 -0.45 0 0 0</pose>
        <collision name="bumper_collision">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>1.1</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>
''')

    parts.append('\n</world>\n</sdf>\n')
    return ''.join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', type=str, required=True,
                    help='scenario name (e.g. 2_rovers_1_humans) '
                         'or its number 1..10')
    ap.add_argument('--out', type=str, default=None,
                    help='output directory (default: worlds/scenarios/<name>)')
    ap.add_argument('--print-name', action='store_true',
                    help='only print the resolved scenario name, then exit')
    args = ap.parse_args()

    wanted = args.scenario.strip()
    if wanted.isdigit():
        scenario_no = int(wanted)
        if scenario_no not in SCENARIOS:
            avail = ', '.join(f'{s} ({SCENARIOS[s]["name"]})'
                              for s in sorted(SCENARIOS))
            sys.exit(f'ERROR: scenario number {scenario_no} not defined '
                     f'(available: {avail})')
    else:
        scenario_no = next((s for s, spec in SCENARIOS.items()
                            if spec['name'] == wanted), None)
        if scenario_no is None:
            avail = ', '.join(spec['name'] for _, spec in sorted(SCENARIOS.items()))
            sys.exit(f'ERROR: scenario {wanted!r} not defined '
                     f'(available: {avail})')

    spec = SCENARIOS[scenario_no]
    if args.print_name:
        print(spec['name'])
        return

    # The numeric id also seeds the small obstacle-placement jitter.
    rng = random.Random(scenario_no)
    humans = build_humans(spec)
    robots = build_robots(spec)
    obstacles = build_obstacles(rng, spec['boxes'], spec['cylinders'],
                                robots, humans)

    out_dir = Path(args.out) if args.out else OUT_ROOT / spec['name']
    out_dir.mkdir(parents=True, exist_ok=True)

    sdf_path = out_dir / 'scenario.sdf'
    json_path = out_dir / 'scenario.json'
    sdf_path.write_text(emit_sdf(scenario_no, robots, humans, obstacles))
    json_path.write_text(json.dumps({
        'scenario': spec['name'],
        'scenario_no': scenario_no,
        'world': WORLD_NAME,
        'robots': robots,
        'humans': humans,
        'obstacles': obstacles,
    }, indent=2) + '\n')

    print(f'scenario {spec["name"]}: {len(robots)} robot(s), '
          f'{len(humans)} human(s), {len(obstacles)} obstacle(s)')
    for r in robots:
        print(f'  {r[0]}: spawn ({r[1]}, {r[2]}) yaw {r[3]}')
    for h in humans:
        print(f'  {h["name"]}: {h["waypoints"][0]} <-> {h["waypoints"][1]} '
              f'(delay {h["delay"]}s)')
    print(f'wrote {sdf_path}')
    print(f'wrote {json_path}')


if __name__ == '__main__':
    main()
