#!/usr/bin/env python3
"""Generate random_world_humans.sdf — random obstacles + STATIONARY humans.

Scenario (for the social-navigation thesis):
  - Same 14x14m arena as random_world (walls, light, physics plugins).
  - `num_boxes` + `num_cylinders` randomly placed static obstacles.
  - `num_humans` STATIONARY humans (static models): each has the stand.dae
    human mesh as the VISUAL (so YOLO detects them) AND a collision cylinder
    (so LiDAR/Nav2 costmap AVOIDS them).  They never move.
  - Positions are seeded (SEED) so every run produces the SAME layout and the
    coordinates are reproducible/reportable.

The robot must navigate between random goals while avoiding BOTH the static
obstacles AND the humans (humans appear as obstacles in the costmap and as
persons to YOLO -> the social yield also engages when close).
"""

import math
import random

# Fixed seed -> deterministic, reportable layout
SEED = 42
random.seed(SEED)

# Configuration
world_file = "random_world_humans.sdf"
world_name = "random_world_humans"
world_size = 7          # half-size of the area inside walls (-7 to 7)
wall_height = 2.0
wall_thickness = 0.5

num_boxes = 5
num_cylinders = 3
num_humans = 7

box_size_range = (0.5, 2.0)
cylinder_radius_range = (0.3, 1.0)
cylinder_height_range = (0.5, 2.0)

# Human model: visual mesh + collision cylinder
ACTOR_MESH_DIR = "/usr/share/gazebo-11/media/models"
STAND_DAE = f"{ACTOR_MESH_DIR}/stand.dae"
HUMAN_COLLISION_RADIUS = 0.3
HUMAN_COLLISION_LENGTH = 1.7

min_distance = 1.5       # min gap between obstacle/human centres

# Robot spawn points to keep clear (robot_0 at origin in this config)
robot_positions = [(0.0, 0.0)]
robot_safe_radius = 0.6

# positions list: (x, y, size) where size ~ diameter of the footprint
positions = []
for rx, ry in robot_positions:
    positions.append((rx, ry, robot_safe_radius))


def is_far_enough(x, y, size):
    for px, py, psize in positions:
        distance = math.hypot(x - px, y - py)
        if distance < (size / 2 + psize / 2 + min_distance):
            return False
    return True


def random_position(size):
    while True:
        x = random.uniform(-world_size + size / 2, world_size - size / 2)
        y = random.uniform(-world_size + size / 2, world_size - size / 2)
        if is_far_enough(x, y, size):
            positions.append((x, y, size))
            return x, y


def random_color():
    colors = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.2, 0.0),
        (0.5, 0.0, 1.0),
        (0.0, 0.8, 1.0),
    ]
    return random.choice(colors)


# Collect coordinates for the report
coords = {"boxes": [], "cylinders": [], "humans": []}

with open(world_file, "w") as f:
    # ---- WORLD HEADER ----
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

    # ---- WALLS ----
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

    # ---- BOXES ----
    for i in range(num_boxes):
        size_x = random.uniform(*box_size_range)
        size_y = random.uniform(*box_size_range)
        size_z = random.uniform(*box_size_range)
        x, y = random_position(max(size_x, size_y))
        z = size_z / 2
        yaw = random.uniform(0, 360)
        r, g, b = random_color()
        coords["boxes"].append((round(x, 2), round(y, 2),
                                round(max(size_x, size_y), 2)))
        f.write(f"""
    <model name="box{i}">
      <pose>{x} {y} {z} 0 0 {math.radians(yaw)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{size_x} {size_y} {size_z}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{size_x} {size_y} {size_z}</size></box>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
      <static>true</static>
    </model>
""")

    # ---- CYLINDERS ----
    for i in range(num_cylinders):
        radius = random.uniform(*cylinder_radius_range)
        height = random.uniform(*cylinder_height_range)
        x, y = random_position(radius * 2)
        z = height / 2
        yaw = random.uniform(0, 360)
        r, g, b = random_color()
        coords["cylinders"].append((round(x, 2), round(y, 2),
                                    round(radius, 2)))
        f.write(f"""
    <model name="cylinder{i}">
      <pose>{x} {y} {z} 0 0 {math.radians(yaw)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>{radius}</radius><length>{height}</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>{radius}</radius><length>{height}</length></cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
      <static>true</static>
    </model>
""")

    # ---- HUMANS (stationary, static models, collision -> Nav2 avoids) ----
    for i in range(num_humans):
        x, y = random_position(HUMAN_COLLISION_RADIUS * 2)
        yaw = random.uniform(0, 360)
        coords["humans"].append((round(x, 2), round(y, 2)))
        f.write(f"""
    <model name="human{i}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {math.radians(yaw)}</pose>
      <link name="body">
        <visual name="visual">
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
              <radius>{HUMAN_COLLISION_RADIUS}</radius>
              <length>{HUMAN_COLLISION_LENGTH}</length>
            </cylinder>
          </geometry>
          <pose>0 0 0.85 0 0 0</pose>
        </collision>
      </link>
    </model>
""")

    # ---- CLOSE WORLD ----
    f.write("""
  </world>
</sdf>
""")

print(f"World file '{world_file}' generated successfully! (seed={SEED})")
print("BOXES (x, y, footprint):", coords["boxes"])
print("CYLINDERS (x, y, radius):", coords["cylinders"])
print("HUMANS (x, y):", coords["humans"])
