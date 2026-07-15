import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('swarm_basics')
    params = os.path.join(pkg, 'config', 'nav2', 'robot_0_nav2.yaml')
    slam_config = os.path.join(pkg, 'config', 'nav2', 'slam_toolbox.yaml')

    return LaunchDescription([
        # === SLAM Toolbox — builds map & localizes on the fly ===
        # No pre-made map needed; drives around to explore the random world
        # Publishes map → odom TF, and occupancy grid on /map
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_config,
                {
                    'map_frame': 'map',
                    'odom_frame': 'robot_0/odom',
                    'base_frame': 'robot_0/base_footprint',
                    'laser_frame': 'robot_0/realsense_link',
                },
            ],
            remappings=[
                ('scan', '/robot_0/scan'),
            ],
            output='screen',
        ),

        # === Nav2 stack ===
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            parameters=[params], output='screen'
        ),

        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            parameters=[params], output='screen',
            remappings=[('cmd_vel', '/robot_0/cmd_vel')]
        ),

        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            parameters=[params], output='screen',
            remappings=[('/odom', '/robot_0/odom')]
        ),

        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            parameters=[params], output='screen'),

        # Lifecycle manager — manages Nav2 nodes (SLAM handles its own lifecycle)
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['planner_server', 'controller_server',
                                        'behavior_server', 'bt_navigator']}],
            output='screen'),
    ])
