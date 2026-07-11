import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('swarm_basics')
    params = os.path.join(pkg, 'config', 'nav2', 'robot_0_nav2.yaml')
    map_yaml = os.path.join(pkg, 'config', 'nav2', 'corridor_map.yaml')

    return LaunchDescription([
        # Map server
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': map_yaml}], output='screen'),
        # AMCL
        Node(
             package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[params], output='screen',
             remappings=[('/scan', '/robot_0/scan')]
             ),
        
        # Planner
        Node(
             package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], output='screen'
             ),

        # Controller
        Node(
             package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], output='screen',
             remappings=[('cmd_vel', '/robot_0/cmd_vel')]
             ),

        # BT Navigator
        Node(
             package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params], output='screen',
             remappings=[('/odom', '/robot_0/odom')]
             ),

        # Behavior server (provides spin, backup, wait actions for BT navigator)
        Node(
             package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[params], output='screen'),

        # Lifecycle manager
        Node(
             package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server',
                                         'behavior_server', 'bt_navigator']}],
             output='screen'),

        # Auto-set initial pose from robot_config.py (delayed so AMCL is ready)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='swarm_basics',
                    executable='set_initial_pose',
                    name='set_initial_pose',
                    parameters=[{'use_sim_time': True}],
                    output='screen'),
            ]),
    ])