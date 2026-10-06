# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from swarm_basics.robot_config import WORLD_NAME


def launch_gz(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = context.perform_substitution(LaunchConfiguration("sim_world"))
    hd = context.perform_substitution(LaunchConfiguration("headless"))

    gz_args = f"-s -r {world}" if hd == "true" else world

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        )
    ]


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("swarm_basics")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_gazebo, "worlds", f"{WORLD_NAME}.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo without GUI (true/false)",
    )

    detector_arg = DeclareLaunchArgument(
        "detector",
        default_value="depth",
        description="Human detector backend: depth (heuristics) or yolo (YOLOv8)",
    )

    enable_llm_arg = DeclareLaunchArgument(
        "enable_llm",
        default_value="true",
        description="Start the llm_planner decision layer (true/false)",
    )

    bt_arg = DeclareLaunchArgument(
        "bt",
        default_value="reactive",
        description="Behavior layer: reactive, ai or stock (bt:=stock logs runs to Nav2/)",
    )

    dynamic_plots_arg = DeclareLaunchArgument(
        "dynamic_plots",
        default_value="true",
        description="Live coverage-map redraws (false = render only at shutdown)",
    )

    log_raw_contacts_arg = DeclareLaunchArgument(
        "log_raw_contacts",
        default_value="false",
        description="Also write every raw contact message to contacts_raw.csv",
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = OpaqueFunction(function=launch_gz)

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_multi_robots.launch.py")
        ),
        launch_arguments={
            "robot_ns": LaunchConfiguration("robot_ns"),
            "detector": LaunchConfiguration("detector"),
            "enable_llm": LaunchConfiguration("enable_llm"),
            "bt": LaunchConfiguration("bt"),
            "dynamic_plots": LaunchConfiguration("dynamic_plots"),
            "log_raw_contacts": LaunchConfiguration("log_raw_contacts"),
        }.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication.
    # The clock's ROS side is REMAPPED to /clock_raw on purpose: Gazebo emits
    # one Clock message per physics step and the bridge forwards every one.
    # MEASURED 2026-09-23 with 4 rovers: 383 Hz.  Every use_sim_time:=true node
    # subscribes, so ~15 nodes/rover x 383 Hz is tens of thousands of callbacks
    # per second whose only job is advancing the clock — enough to starve TF,
    # slam_toolbox and the Nav2 lifecycle service calls (symptoms: costmaps
    # stuck in 'Activating' with their clock frozen at 0.7 s, 'queue is full'
    # scan drops, 'get_state' timeouts aborting bringup).  clock_throttle
    # republishes it as /clock at a sane rate.
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[("/clock", "/clock_raw")],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # /clock_raw (physics rate, ~383 Hz) -> /clock (SWARM_CLOCK_HZ, default
    # 50 Hz).  Sim time only needs to advance smoothly: control runs at 20 Hz
    # and the Nav2 BT tick at 100 Hz, so 50 Hz costs nothing and removes ~87 %
    # of the clock traffic.  0 disables the relay (clock stays on /clock_raw and
    # nothing has sim time — only useful for diagnosing the bridge itself).
    clock_hz = float(os.environ.get("SWARM_CLOCK_HZ", "50") or 50.0)
    clock_relay = Node(
        package="swarm_basics",
        executable="clock_throttle",
        name="clock_throttle",
        parameters=[{
            "input_topic": "/clock_raw",
            "output_topic": "/clock",
            "rate_hz": clock_hz if clock_hz > 0 else 50.0,
        }],
        output="screen",
    )

    return LaunchDescription(
        [
            sim_world,
            robot_ns,
            headless_arg,
            detector_arg,
            enable_llm_arg,
            bt_arg,
            dynamic_plots_arg,
            log_raw_contacts_arg,
            gz_sim,
            spawn_robot,
            topic_bridge,
            clock_relay,
        ]
    )
