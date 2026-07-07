import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_swarm_basics = get_package_share_directory("swarm_basics")

    # --- SLAM Toolbox config ---
    slam_config = os.path.join(pkg_swarm_basics, "config", "nav2", "slam_toolbox.yaml")

    # --- SLAM Toolbox for robot_0 only ---
    # Uses odom_frame="odom" to match Gazebo's diff-drive plugin frame_id.
    # Only one robot runs SLAM for now to avoid TF conflicts on map→odom.
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_config,
            {
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "robot_0/base_footprint",
                "laser_frame": "robot_0/realsense_link",
                "scan_topic": "/robot_0/scan",
            },
        ],
        output="screen",
    )

    # Ground truth map overlay — shows corridor walls + cube in RViz
    ground_truth = Node(
        package="swarm_basics",
        executable="ground_truth_map",
        name="ground_truth_map",
        output="screen",
    )

    return LaunchDescription([
        slam_node,
        ground_truth,
    ])
