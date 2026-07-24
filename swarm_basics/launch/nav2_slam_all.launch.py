"""SLAM + Nav2 for ALL robots — reads nav2_generic.yaml, merges per-robot frames."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from swarm_basics.robot_config import ROBOT_POSITIONS


def load_common():
    """Load params from nav2_generic.yaml, strip the /**/ros__parameters wrapper."""
    pkg = get_package_share_directory('swarm_basics')
    path = os.path.join(pkg, 'config', 'nav2', 'nav2_generic.yaml')
    with open(path) as f:
        raw = yaml.safe_load(f)
    # raw = {'/**': {'ros__parameters': {...}}}
    return raw.get('/**', {}).get('ros__parameters', {})


def per_robot_params(ns):
    """Return a single flat dict: common + per-robot overrides."""
    bf = f'{ns}/base_footprint'
    of = f'{ns}/odom'
    scan = f'/{ns}/lidar/scan'

    p = load_common()

    # Global costmap: shared 'map' frame and topic
    p['global_costmap.global_costmap.ros__parameters.global_frame'] = 'map'
    p['global_costmap.global_costmap.ros__parameters.robot_base_frame'] = bf
    p['global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic'] = scan
    p['global_costmap.global_costmap.ros__parameters.static_layer.map_topic'] = '/map'
    p['local_costmap.local_costmap.ros__parameters.global_frame'] = of
    p['local_costmap.local_costmap.ros__parameters.robot_base_frame'] = bf
    p['local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic'] = scan

    # Controller local costmap
    p['local_costmap.local_costmap.ros__parameters.global_frame'] = of
    p['local_costmap.local_costmap.ros__parameters.robot_base_frame'] = bf
    p['local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic'] = scan

    # BT Navigator
    p['global_frame'] = of
    p['robot_base_frame'] = bf
    p['odom_frame'] = of

    # Behavior Server
    p['global_frame'] = of
    p['robot_base_frame'] = bf

    return p


def create_slam_nodes(context, robots):
    pkg = get_package_share_directory('swarm_basics')
    slam_config = os.path.join(pkg, 'config', 'nav2', 'slam_toolbox.yaml')

    nodes = []

    for ns, _, _, _ in robots:
        params = per_robot_params(ns)
        scan_topic = f'/{ns}/lidar/scan'
        cmd_vel_topic = f'/{ns}/cmd_vel'
        odom_topic = f'/{ns}/odom'
        # Gazebo scan frame is {ns}/{ns}/base_footprint/lidar — match it exactly
        gz_lidar_frame = f'{ns}/{ns}/base_footprint/lidar'
        laser_frame = gz_lidar_frame

        # Static TF: bridge whatever Gazebo produces to base_footprint
        # Direction: base_footprint → gz_frame (so gz_frame is child, won't conflict with odom→base_footprint)
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'{ns}_lidar_frame_bridge',
            arguments=['0', '0', '0.1', '0', '0', '0',
                       f'{ns}/base_footprint', f'{ns}/{ns}/base_footprint/lidar'],
        ))

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
