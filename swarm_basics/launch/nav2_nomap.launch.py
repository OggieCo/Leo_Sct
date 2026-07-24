"""Nav2 without a map — local obstacle avoidance only.
Uses odom as global frame, rolling costmap, camera-based scan from depth_to_scan.
Send goals relative to the robot's starting position via /navigate_to_pose action.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('swarm_basics')
    params = os.path.join(pkg, 'config', 'nav2', 'robot_0_nav2_nomap.yaml')

    return LaunchDescription([
        # Planner
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            parameters=[params], output='screen'
        ),

        # Controller (local planner, obstacle avoidance)
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            parameters=[params], output='screen',
            remappings=[('cmd_vel', '/robot_0/cmd_vel')]
        ),

        # BT Navigator (receives /navigate_to_pose goals)
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            parameters=[params], output='screen',
            remappings=[('/odom', '/robot_0/odom')]
        ),

        # Behavior server (spin, backup, wait)
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            parameters=[params], output='screen'),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['planner_server', 'controller_server',
                                        'behavior_server', 'bt_navigator']}],
            output='screen'),
    ])
