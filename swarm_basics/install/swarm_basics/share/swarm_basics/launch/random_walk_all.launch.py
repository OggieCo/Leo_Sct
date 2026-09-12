"""Run random_walk for EVERY robot in robot_config.py — one command for the whole swarm."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from swarm_basics.robot_config import ROBOT_POSITIONS


def create_walk_nodes(context, robots):
    nodes = []
    for ns, _, _, _ in robots:
        nodes.append(Node(
            package='swarm_basics',
            executable='random_walk',
            name='random_walk',
            namespace=ns,
            output='screen',
        ))
    return nodes


def generate_launch_description():
    robot_list = [(ns, x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS]
    return LaunchDescription([
        OpaqueFunction(function=lambda ctx: create_walk_nodes(ctx, robot_list)),
    ])
