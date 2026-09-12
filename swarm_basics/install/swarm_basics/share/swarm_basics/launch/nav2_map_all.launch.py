"""Known-map Nav2 for ALL robots — map_server + AMCL instead of SLAM.

Same structure as nav2_slam_all.launch.py, but the map is NOT built live:
each robot gets its own map_server (loading the SAME pre-made map into its
/{ns}/map) and its own AMCL (localizing against it and publishing the
/{ns}/map -> /{ns}/odom transform).

Use this when you have a saved map of the world (e.g. corridor_map.yaml, or a
map exported from a SLAM run with nav2_map_saver). For unknown environments use
nav2_slam_all.launch.py instead.

Run:
    ros2 launch swarm_basics nav2_map_all.launch.py
    # or with a specific map:
    ros2 launch swarm_basics nav2_map_all.launch.py map:=/path/to/map.yaml
"""

import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
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
    mapf = f'{ns}/map'
    scan = f'/{ns}/lidar/scan_clean'

    p = load_common()

    # ---- Global costmap: per-robot map frame + per-robot map topic ----
    # The static layer subscribes to /{ns}/map, which map_server publishes.
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    gc['global_frame'] = mapf
    gc['robot_base_frame'] = bf
    gc['static_layer']['map_topic'] = f'/{ns}/map'
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


def write_costmap_params_file(ns, params):
    """Write ONLY the costmap sections to a temp YAML with the namespace as root key.

    Official Nav2 multi-robot pattern: embedded costmap nodes (child LifecycleNodes
    of planner/controller) inherit the parent's params file and match their section
    by fully-qualified name (e.g. robot_0/global_costmap/global_costmap). Passing the
    sections as a real file (not a dict) makes rclcpp place them at the right path.
    """
    costmap_only = {
        ns: {
            'global_costmap': params['global_costmap'],
            'local_costmap': params['local_costmap'],
        }
    }
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix=f'{ns}_costmap_')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(costmap_only, f)
    return path


def create_map_nodes(context, robots, map_yaml):
    nodes = []

    for ns, _, _, _ in robots:
        params = per_robot_params(ns)
        costmap_file = write_costmap_params_file(ns, params)
        # Clean scan topic: {ns}/lidar/scan_clean is republished with frame {ns}/lidar_link
        scan_topic = f'/{ns}/lidar/scan_clean'
        cmd_vel_topic = f'/{ns}/cmd_vel'
        odom_topic = f'/{ns}/odom'

        # Map server — loads the pre-made map, publishes it on /{ns}/map (Transient Local)
        nodes.append(Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', namespace=ns,
            parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
            output='screen',
        ))

        # AMCL — localizes against the static map, publishes /{ns}/map -> /{ns}/odom.
        # robot_0's initial pose arrives on /initialpose (see set_initial_pose),
        # everyone else's on /{ns}/initialpose (namespaced automatically).
        nodes.append(Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'base_frame_id': f'{ns}/base_footprint',
                'global_frame_id': f'{ns}/map',
                'odom_frame_id': f'{ns}/odom',
                'scan_topic': scan_topic,
                'transform_tolerance': 0.5,
            }],
            remappings=[('initialpose', '/initialpose')] if ns == 'robot_0' else [],
            output='screen',
        ))

        # Planner — namespace isolates /{ns}/map, /{ns}/plan, etc.
        nodes.append(Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', namespace=ns,
            parameters=[params, costmap_file],
            output='screen',
        ))

        # Controller
        nodes.append(Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', namespace=ns,
            parameters=[params, costmap_file],
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

        # Lifecycle Manager — manages map_server + amcl + Nav2 servers.
        # use_sim_time=False: the manager only orchestrates lifecycle transitions
        # + bonds; running it on wall clock avoids the classic sim-time stall.
        # Delayed start + generous bond_timeout: lets AMCL publish {ns}/map->{ns}/odom
        # (and lidar_republish/odom_tf) come up FIRST, so embedded costmaps can
        # resolve their initial transform and Nav2 activates cleanly.
        nodes.append(TimerAction(
            period=12.0,
            actions=[Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager', namespace=ns,
                parameters=[{'use_sim_time': False, 'autostart': True,
                             'bond_timeout': 20.0,
                             'attempt_respawn_reconnection': True,
                             'node_names': ['map_server', 'amcl', 'planner_server',
                                            'controller_server', 'behavior_server',
                                            'bt_navigator']}],
                output='screen',
            )],
        ))

    # Set initial pose for ALL robots (single node; publishes /initialpose for
    # robot_0 and /{ns}/initialpose for the rest — AMCL picks them up).
    # Delayed until AMCL is active (it waits up to 10 s for a subscriber anyway).
    nodes.append(TimerAction(
        period=15.0,
        actions=[Node(
            package='swarm_basics', executable='set_initial_pose',
            name='set_initial_pose',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )],
    ))

    # Viz-only connector: align every robot's map frame to robot_0/map so RViz can
    # display all rovers in one view. Does NOT affect navigation — each robot still
    # localizes in its own {ns}/map frame.
    origin = None
    for r in robots:
        if r[0] == 'robot_0':
            origin = (r[1], r[2], r[3])
    if origin:
        x0, y0, yaw0 = origin
        for r in robots:
            if r[0] == 'robot_0':
                continue
            ns, x, y, yaw = r
            nodes.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=f'{ns}_map_connector',
                arguments=[str(x - x0), str(y - y0), '0.0',
                           str(yaw - yaw0), '0.0', '0.0',
                           'robot_0/map', f'{ns}/map'],
                output='screen',
            ))

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('swarm_basics')
    default_map = os.path.join(pkg, 'config', 'nav2', 'corridor_map.yaml')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to the pre-made map YAML (image + pgm next to it).',
    )
    robot_list = [(ns, x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS]

    def build(ctx):
        map_yaml = LaunchConfiguration('map').perform(ctx)
        return create_map_nodes(ctx, robot_list, map_yaml)

    return LaunchDescription([
        map_arg,
        OpaqueFunction(function=build),
    ])
