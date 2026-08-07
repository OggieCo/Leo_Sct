"""Run random_goals (Nav2-driven) for EVERY robot in robot_config.py — one command
for the whole swarm. Nav2 goals use the per-robot costmap, so rovers avoid the
static human automatically."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from swarm_basics.robot_config import ROBOT_POSITIONS


def create_goal_nodes(context, robots):
    nodes = []
    for ns, _, _, _ in robots:
        nodes.append(Node(
            package='swarm_basics',
            executable='random_goals',
            name='random_goals',
            namespace=ns,                          # unique /robot_X/random_goals
            parameters=[{'robot_ns': ns}],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    robot_list = [(ns, x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS]
    return LaunchDescription([
        OpaqueFunction(function=lambda ctx: create_goal_nodes(ctx, robot_list)),
    ])
