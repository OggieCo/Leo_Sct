#!/usr/bin/env python3
"""Generate human_world.sdf — a clean arena with a STATIONARY human standing in
front of the rovers.

Scenario (for the social-navigation thesis):
  - Same 14x14m arena as before (walls, light, plugins).
  - ONE human standing still directly in front of the rovers' spawn.
  - The human is a STATIC MODEL WITH COLLISION (cylinder), NOT a Gazebo actor,
    so the rovers' LiDAR detects it -> obstacle layer in the costmap -> Nav2
    plans around it (i.e. the rovers SEE it and AVOID it).

Moving actors (walking humans) are kept behind the INCLUDE_WALKERS flag — they
are kinematic (no collision), so they are only visible to the camera, not to
Nav2. Re-enable them for the full social scenario later.
"""

import math
import time

random_seed = time.time()
import random
random.seed(random_seed)

# Configuration
world_file = "human_world.sdf"
world_name = "human_world"
world_size = 7          # half-size of the area inside walls (-7 to 7)
wall_height = 2.0
wall_thickness = 0.5

# Stationary human (static model with collision -> LiDAR/Nav2 can avoid it)
INCLUDE_STATIONARY_HUMAN = False     # OFF while the wait-and-see actor is ON
STATIONARY_POS = (3.0, 0.0, 1.5708)  # directly in front of rovers; yaw +90° (turned left)
# (spawn at 0,0 / 1,0, facing +x)

# "Wait-and-see" human (Gazebo ACTOR): stands still in front of the rovers for
# HUMAN_STAND_S seconds, then walks across and loops.  Lets us test: rover
# stops for the human; if the human moves before the rover's deadline, the
# rover proceeds forward instead of detouring.  Kinematic -> camera-only.
INCLUDE_WAITING_HUMAN = True
WAITING_HUMAN_POS = (3.0, 0.0)
HUMAN_STAND_S = 30.0      # s standing still — long enough for the rover to reach it while standing
WALK_SPEED = 0.6          # m/s once it starts walking

# Moving actors (kinematic, camera-only, NOT avoidable by Nav2)
INCLUDE_WALKERS = False
num_walkers = 3         # slow walkers (~0.4 m/s)
num_fast = 1            # fast crosser (~1.0 m/s)

# Gazebo actor meshes — available locally in the container (no internet needed)
ACTOR_MESH_DIR = "/usr/share/gazebo-11/media/models"
WALK_DAE = f"{ACTOR_MESH_DIR}/walk.dae"
RUN_DAE = f"{ACTOR_MESH_DIR}/run.dae"
STAND_DAE = f"{ACTOR_MESH_DIR}/stand.dae"
ACTOR_Z = 1.0  # actor body centre height (feet at ground ~0)

HUMAN_CLEARANCE = 0.5   # min distance from arena walls while walking
ARENA_LIMIT = world_size - 0.8


def write_actor(f, name, mesh, waypoints, delay=1.0):
    anim = "walk" if "walk" in mesh else "run"
    f.write(f"""
    <actor name="{name}">
      <skin>
        <filename>{mesh}</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="{anim}">
        <filename>{mesh}</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>{delay}</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="LINEAR">
""")
    for t, x, y in waypoints:
        f.write(f"""          <waypoint><time>{t}</time><pose>{x} {y} {ACTOR_Z} 0 0 0</pose></waypoint>
""")
    f.write("""        </trajectory>
      </script>
    </actor>
""")


def write_static_human(f, name, x, y, yaw=0.0):
    """A standing human as a STATIC MODEL with a collision cylinder.

    The stand.dae mesh gives the visual human shape; the cylinder is what the
    LiDAR raycasts hit, so the costmap obstacle layer sees it and Nav2 avoids it.
    """
    f.write(f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {yaw}</pose>
      <link name="body">
        <visual name="visual">
          <!-- stand.dae's origin is at the BODY CENTRE (~1 m), so lift the mesh
               so the FEET sit on the ground (z=0). Without this the human
               appears half-buried in the floor. -->
          <pose>0 0 1.0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>file://{STAND_DAE}</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.7</length>
            </cylinder>
          </geometry>
          <pose>0 0 0.85 0 0 0</pose>
        </collision>
      </link>
    </model>
""")


def write_waiting_human(f, name, x, y, stand_s=12.0, speed=0.6):
    """A Gazebo ACTOR that stands still at (x,y) for `stand_s` seconds, then
    walks back and forth across the arena in front of the rover.

    Kinematic (no collision) -> visible to the CAMERA only, which is exactly
    what the wait-and-see yield test needs: the rover reacts to the camera's
    human_close, and "the human moved" == human_close going false.
    """
    half = 3.0              # walks ±3 m along y from its standing spot
    t_walk = 2 * half / speed  # time to cross one way
    f.write(f"""
    <actor name="{name}">
      <skin>
        <filename>{WALK_DAE}</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>{WALK_DAE}</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>1.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="LINEAR">
          <waypoint><time>0</time><pose>{x} {y} {ACTOR_Z} 0 0 0</pose></waypoint>
          <waypoint><time>{stand_s}</time><pose>{x} {y} {ACTOR_Z} 0 0 0</pose></waypoint>
          <waypoint><time>{stand_s + t_walk}</time><pose>{x} {y - half} {ACTOR_Z} 0 0 0</pose></waypoint>
          <waypoint><time>{stand_s + 2 * t_walk}</time><pose>{x} {y + half} {ACTOR_Z} 0 0 0</pose></waypoint>
          <waypoint><time>{stand_s + 3 * t_walk}</time><pose>{x} {y} {ACTOR_Z} 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
""")


def loop_waypoints(x1, y1, x2, y2, speed, t0=0.0):
    """Timed waypoints going forward then back (smooth loop). speed in m/s."""
    fwd = []
    t = t0
    # forward
    d = math.hypot(x2 - x1, y2 - y1)
    fwd.append((t, x1, y1))
    t += d / speed
    fwd.append((t, x2, y2))
    t += d / speed
    fwd.append((t, x1, y1))
    return fwd


with open(world_file, "w") as f:
    # ---- WORLD HEADER (same as random_world) ----
    f.write(f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{world_name}">
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
    <plugin
      filename="libignition-gazebo-contact-system.so"
      name="ignition::gazebo::systems::Contact">
      <topic>/world/{world_name}/physics/contacts</topic>
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
""")

    # ---- WALLS (same as random_world) ----
    walls = [
        ("north_wall", 0, world_size + wall_thickness / 2, 20, wall_thickness),
        ("south_wall", 0, -world_size - wall_thickness / 2, 20, wall_thickness),
        ("east_wall", world_size + wall_thickness / 2, 0, wall_thickness, 20),
        ("west_wall", -world_size - wall_thickness / 2, 0, wall_thickness, 20),
    ]
    for name, x, y, size_x, size_y in walls:
        z = wall_height / 2
        f.write(f"""
    <model name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{size_x} {size_y} {wall_height}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{size_x} {size_y} {wall_height}</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
      <static>true</static>
    </model>
""")

    # ---- STATIONARY HUMAN (static model with collision, in front of rovers) ----
    if INCLUDE_STATIONARY_HUMAN and not INCLUDE_WAITING_HUMAN:
        sx, sy, syaw = STATIONARY_POS
        write_static_human(f, "human_static", sx, sy, syaw)

    # ---- WAITING HUMAN (actor: stands still, then walks away) ----
    if INCLUDE_WAITING_HUMAN:
        wx, wy = WAITING_HUMAN_POS
        write_waiting_human(f, "human_waiting", wx, wy, HUMAN_STAND_S, WALK_SPEED)

    # ---- MOVING ACTORS (kinematic, camera-only) — disabled by default ----
    if INCLUDE_WALKERS:
        # Straight crossings in the empty arena — no obstacles to avoid.
        # Slow walkers: ~0.4 m/s crossing the arena (14 m round trip ~35 s)
        write_actor(f, "human_slow_0", WALK_DAE,
                    loop_waypoints(-5.0, -3.0, 5.0, -3.0, 0.4))
        write_actor(f, "human_slow_1", WALK_DAE,
                    loop_waypoints(3.0, -5.0, 3.0, 5.0, 0.4))
        write_actor(f, "human_slow_2", WALK_DAE,
                    loop_waypoints(-5.0, 4.0, 5.0, 4.0, 0.4), delay=5.0)

        # Fast crosser: ~1.0 m/s
        write_actor(f, "human_fast_0", RUN_DAE,
                    loop_waypoints(-6.0, 5.0, 6.0, 5.0, 1.0), delay=8.0)

    # ---- CLOSE WORLD ----
    f.write("""
  </world>
</sdf>
""")

print(f"World file '{world_file}' generated successfully!")
