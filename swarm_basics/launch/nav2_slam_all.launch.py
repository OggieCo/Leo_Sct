"""SLAM + Nav2 for ALL robots — reads nav2_generic.yaml, merges per-robot frames."""

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


def create_slam_nodes(context, robots, bt='reactive'):
    pkg = get_package_share_directory('swarm_basics')
    slam_config = os.path.join(pkg, 'config', 'nav2', 'slam_toolbox.yaml')
    bt_name = 'social_nav.xml' if bt != 'ai' else 'ai_nav.xml'
    bt_xml = f'/root/ros2_ws/src/swarm_basics/config/bt/{bt_name}'

    nodes = []

    for ns, _, _, _ in robots:
        params = per_robot_params(ns)
        # Which behavior tree: reactive (social_nav.xml) or AI (ai_nav.xml)
        params['default_nav_to_pose_bt_xml'] = bt_xml
        # Costmap sections must reach the embedded costmaps as a real YAML file
        # (namespace root key), not via the dict. Flat server params stay in dict.
        costmap_file = write_costmap_params_file(ns, params)
        # Clean scan topic: {ns}/lidar/scan_clean is republished with frame {ns}/lidar_link
        # (lidar_republish node fixes Gazebo's mangled frame)
        scan_topic = f'/{ns}/lidar/scan_clean'
        # Controller -> velocity_adaptor -> robot driver (social speed slowing)
        cmd_vel_topic = f'/{ns}/cmd_vel_social'
        odom_topic = f'/{ns}/odom'
        laser_frame = f'{ns}/lidar_link'

        # SLAM Toolbox — publishes {ns}/map→{ns}/odom, occupancy grid on /{ns}/map (per-robot)
        nodes.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name=f'{ns}_slam_toolbox',
            parameters=[slam_config, {
                'map_frame': f'{ns}/map', 'odom_frame': f'{ns}/odom',
                'base_frame': f'{ns}/base_footprint', 'laser_frame': laser_frame,
            }],
            remappings=[('scan', scan_topic), ('map', f'/{ns}/map')],
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

        # Lifecycle Manager — same namespace, uses bare node names.
        # use_sim_time=False: the manager only orchestrates lifecycle
        # transitions + bonds; running it on wall clock avoids the classic
        # sim-time stall where change_state service timeouts freeze the whole
        # bringup (seen as a 60s+ hang right after "Configuring planner_server"
        # with a "change_state (timeout)" warning). The Nav2 servers below still
        # use sim time for transforms.
        # Delayed start + generous bond_timeout: lets SLAM publish {ns}/map→{ns}/odom
        # (and lidar_republish/odom_tf) come up FIRST, so embedded costmaps can
        # resolve their initial transform and Nav2 activates cleanly. bond_timeout
        # > 4s default so a slow-configured node doesn't cause the manager to give
        # up mid-bring-up (which left a robot's bt_navigator inactive before).
        nodes.append(TimerAction(
            period=12.0,
            actions=[Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager', namespace=ns,
                parameters=[{'use_sim_time': False, 'autostart': True,
                             'bond_timeout': 20.0,
                             'attempt_respawn_reconnection': True,
                             'node_names': ['planner_server', 'controller_server',
                                            'behavior_server', 'bt_navigator']}],
                output='screen',
            )],
        ))

    # Viz-only connector: align every robot's map frame to robot_0/map so RViz can
    # display all rovers in one view. Does NOT affect navigation — each robot still
    # plans in its own {ns}/map frame.
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
    bt_arg = DeclareLaunchArgument(
        'bt', default_value='reactive',
        description='Behavior tree: reactive (social_nav.xml) or ai (ai_nav.xml)')
    bt = LaunchConfiguration('bt')
    robot_list = [(ns, x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS]
    return LaunchDescription([
        bt_arg,
        OpaqueFunction(function=lambda ctx: create_slam_nodes(
            ctx, robot_list, ctx.perform_substitution(bt))),
    ])
