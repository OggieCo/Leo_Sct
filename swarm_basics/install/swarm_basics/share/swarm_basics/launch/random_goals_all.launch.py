"""Run random_goals (Nav2-driven) for ALL robots in robot_config.py — one command
for the whole swarm. A single random_goals node manages 1..N robots; each robot
gets a new random goal only when it reaches its own previous goal. Nav2 goals use
the per-robot costmap, so rovers avoid the static human automatically."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_basics',
            executable='random_goals',
            name='random_goals',
            output='screen',
        ),
    ])
