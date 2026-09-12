"""Run the FIXED goal tour (fixed_goals.py) for ALL robots in robot_config.py —
one command for the whole swarm, identical goals for every condition so the
BT vs BT+LLM A/B is not luck-based.  Each robot walks the same FIXED_GOALS
sequence (see fixed_goals.py) one goal at a time.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_basics',
            executable='fixed_goals',
            name='fixed_goals',
            output='screen',
        ),
    ])
