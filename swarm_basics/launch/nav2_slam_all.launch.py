"""SLAM + Nav2 for ALL robots — reads nav2_generic.yaml, merges per-robot frames."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from swarm_basics.robot_config import ROBOT_POSITIONS


def load_common():
    """Load params from nav2_generic.yaml: /** flat params + nested costmap sections."""
    pkg = get_package_share_directory('swarm_basics')
    path = os.path.join(pkg, 'config', 'nav2', 'nav2_generic.yaml')
    with open(path) as f:
        raw = yaml.safe_load(f)

    params = dict(raw.get('/**', {}).get('ros__parameters', {}))
    # Costmap config lives in top-level nested sections — merge them in so the
    # costmap sub-nodes receive plugins/layers/frames (not just defaults).
    for section in ('global_costmap', 'local_costmap'):
        if section in raw:
            params[section] = raw[section]
    return params


def per_robot_params(ns):
    """Return a single dict: common + per-robot overrides (nested for costmaps)."""
    bf = f'{ns}/base_footprint'
    of = f'{ns}/odom'
    scan = f'/{ns}/lidar/scan_clean'

    p = load_common()

    # ---- Global costmap: shared 'map' frame + shared map topic ----
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    gc['global_frame'] = 'map'
    gc['robot_base_frame'] = bf
    gc['static_layer']['map_topic'] = '/map'
    gc['obstacle_layer']['scan']['topic'] = scan

    # ---- Local costmap ----
    lc = p['local_costmap']['local_costmap']['ros__parameters']
    lc['global_frame'] = of
    lc['robot_base_frame'] = bf
    lc['obstacle_layer']['scan']['topic'] = scan

    # ---- BT Navigator ----
    p['global_frame'] = of
    p['robot_base_frame'] = bf
    p['odom_frame'] = of

    # ---- Behavior Server ----
    p['global_frame'] = of
    p['robot_base_frame'] = bf

    return p


def create_slam_nodes(context, robots):
    pkg = get_package_share_directory('swarm_basics')
    slam_config = os.path.join(pkg, 'config', 'nav2', 'slam_toolbox.yaml')

    nodes = []

    for ns, _, _, _ in robots:
        params = per_robot_params(ns)
        # Clean scan topic: {ns}/lidar/scan_clean is republished with frame {ns}/lidar_link
        # (lidar_republish node fixes Gazebo's mangled frame)
        scan_topic = f'/{ns}/lidar/scan_clean'
        cmd_vel_topic = f'/{ns}/cmd_vel'
        odom_topic = f'/{ns}/odom'
        laser_frame = f'{ns}/lidar_link'

        # SLAM Toolbox — publishes map→{ns}/odom, occupancy grid on /map (shared)
        nodes.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name=f'{ns}_slam_toolbox',
            parameters=[slam_config, {
                'map_frame': 'map', 'odom_frame': f'{ns}/odom',
                'base_frame': f'{ns}/base_footprint', 'laser_frame': laser_frame,
            }],
            remappings=[('scan', scan_topic)],
            output='screen',
        ))

        # Planner — namespace isolates /{ns}/map, /{ns}/plan, etc.
        nodes.append(Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', namespace=ns,
            parameters=[params],
            output='screen',
        ))

        # Controller
        nodes.append(Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', namespace=ns,
            parameters=[params],
            remappings=[('cmd_vel', cmd_vel_topic)],
            output='screen',
        ))

        # BT Navigator
        nodes.append(Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', namespace=ns,
            parameters=[params],
            remappings=[('/odom', odom_topic)],
            output='screen',
        ))

        # Behavior Server
        nodes.append(Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', namespace=ns,
            parameters=[params],
            output='screen',
        ))

        # Lifecycle Manager — same namespace, uses bare node names
        nodes.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager', namespace=ns,
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['planner_server', 'controller_server',
                                        'behavior_server', 'bt_navigator']}],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    robot_list = [(ns, x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS]
    return LaunchDescription([
        OpaqueFunction(function=lambda ctx: create_slam_nodes(ctx, robot_list)),
    ])
