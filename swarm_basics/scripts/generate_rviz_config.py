#!/usr/bin/env python3
"""Generate an RViz2 config that shows every rover in ROBOT_POSITIONS.

Displays per robot: RobotModel, SLAM Map, LaserScan and global costmap.
Usage:  python3 generate_rviz_config.py [output_path]
"""

import os
import sys

# allow importing robot_config from the package root
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from swarm_basics.robot_config import ROBOT_POSITIONS  # noqa: E402


def _map_display(name, topic, color_scheme, alpha):
    return f"""    - Alpha: {alpha}
      Class: rviz_default_plugins/Map
      Color Scheme: {color_scheme}
      Draw Behind: false
      Enabled: true
      Name: {name}
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {topic}
      Use Timestamp: false
      Value: true"""


def _scan_display(name, topic, color):
    return f"""    - Class: rviz_default_plugins/LaserScan
      Color: {color}
      Color_Preset: Intensity
      Enabled: true
      Name: {name}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: {topic}
      Use Fixed Frame: true
      Value: true"""


def _model_display(name, topic):
    return f"""    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {topic}
      Enabled: true
      Name: {name}
      Update Interval: 0
      Value: true
      Visual Enabled: true"""


def build(robots):
    namespaces = [ns for ns, _, _, _ in robots]
    fixed = f'{namespaces[0]}/map' if namespaces else 'map'
    colors = ['255; 0; 0', '0; 255; 0', '0; 0; 255', '255; 255; 0',
              '255; 0; 255', '0; 255; 255']

    displays = []
    displays.append("""    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 30
      Reference Frame: <Fixed Frame>
      Value: true""")
    displays.append("""    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.35
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree: {}
      Update Interval: 0
      Value: true""")

    for i, ns in enumerate(namespaces):
        col = colors[i % len(colors)]
        displays.append(_model_display(f'Robot {ns}', f'/{ns}/robot_description'))
        displays.append(_map_display(f'Map {ns}', f'/{ns}/map', 'map', 0.7))
        displays.append(_scan_display(f'Laser {ns}', f'/{ns}/lidar/scan_clean', col))
        displays.append(_map_display(f'Costmap {ns}', f'/{ns}/global_costmap/costmap', 'costmap', 0.5))

    body = '\n'.join(displays)
    return f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 646
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
{body}
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {fixed}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 12
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.785
      Roll: 0
      Yaw: 0
    Saved: ~
"""


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, '..', 'config', 'swarm_map_view.rviz')
    robots = [(ns, x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS]
    with open(out, 'w') as f:
        f.write(build(robots))
    print(f'Wrote {os.path.abspath(out)} for robots: '
          f'{", ".join(ns for ns, *_ in robots) or "(none)"}')


if __name__ == '__main__':
    main()
