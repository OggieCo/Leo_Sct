import os
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Load robot positions from shared config (edit robot_config.py to add/remove robots)
from swarm_basics.robot_config import ROBOT_POSITIONS, WORLD_NAME

def generate_launch_description():

    # detector: 'depth' (image_human_processor heuristics) or 'yolo' (YOLOv8)
    detector_arg = DeclareLaunchArgument(
        'detector', default_value='depth',
        description='Human detector backend: depth or yolo')

    # LLM: 'true' -> llm_planner runs (BT executes LLM decisions, reactive
    # rules stay as fallback). 'false' -> pure reactive BT (no LLM topic).
    enable_llm_arg = DeclareLaunchArgument(
        'enable_llm', default_value='true',
        description='Start the llm_planner decision layer (true/false)')

    leo_description = get_package_share_directory("leo_description")

    robots_to_spawn = []
    for ns, x, y, yaw in ROBOT_POSITIONS:
        robots_to_spawn.append({
            "ns": ns,
            "x": x,
            "y": y,
            "yaw": yaw
        })

    plot_node = Node(
            package="swarm_basics",
            executable="coverage_plotter",
            name="coverage_plotter",
            output="screen"
    )       

    # Social/behavior CSV logger — writes social_state.csv + social_events.csv
    # into the same run folder as coverage_plotter.
    social_logger_node = Node(
            package="swarm_basics",
            executable="social_event_logger",
            name="social_event_logger",
            output="screen"
    )

    detector = LaunchConfiguration('detector')
    enable_llm = LaunchConfiguration('enable_llm')

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context, robots):
        nodes = []
        det = context.perform_substitution(detector)   # 'depth' or 'yolo'
        llm_on = context.perform_substitution(enable_llm).lower() in ('true', '1')

        # --- One bridge for all robots ---
        bridge_args = []
        for robot in robots:
            ns = robot["ns"]
            bridge_args += [
                # ⬆ OUT: supervisor sends motion → bridge feeds it to Gazebo → robot drives
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",

                # ⬇ IN: Gazebo tells where robot is → bridge publishes → odom_tf_publisher & RViz read it
                f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",

                # ⬇ IN: Gazebo broadcasts all model positions → bridge → root /tf → RViz draws the world
                f"/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",

                # ⬇ IN: Gazebo LiDAR scan (360°) → bridge → /robot_0/scan for Nav2 SLAM & costmap
                f"/{ns}/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",

                # ⬇ IN: Gazebo raw depth data → bridge → image_processor & LLM camera feed
                f"/{ns}/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",

                # ⬇ IN: Gazebo camera calibration → bridge → image_processor
                f"/{ns}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",

                # ⬇ IN: Gazebo rgb image → bridge → image_processor & LLM vision
                f"/{ns}/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",

                # ⬇ IN: Gazebo collision sensor → bridge → bump_counter logs "ouch, I hit something"
                f"/world/{WORLD_NAME}/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"
                f"@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",

                # ⬇ IN: Gazebo wheel positions → bridge → robot_state_publisher → TF → RViz shows spinning wheels
                f"/{ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ]

        # ⬇ IN: Global pose topic for coverage_plotter
        bridge_args += [
            f"/world/{WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
        ]    

        bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="all_robots_bridge",
            arguments=bridge_args,
            parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
            output="screen"
        )
        nodes.append(bridge_node)

        # --- Create each robot ---
        for robot in robots:
            ns = robot["ns"]
            x = robot["x"]
            y = robot["y"]
            yaw = robot["yaw"]

            # URDF with per-robot namespace mapping
            xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')
            doc = xacro.process_file(xacro_file, mappings={"robot_ns": ns})
            robot_description = doc.toxml()

            # State publisher (per robot)
            state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True,
                    "robot_description": robot_description
                }],
                #remappings=[("/joint_states", f"{ns}/joint_states")],
                output="screen"
            )

            # Spawn robot in Gazebo
            spawn_node = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=ns,
                arguments=[
                    "-name", ns,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", "0.1",
                    "-Y", str(yaw),
                    "-topic", f"/{ns}/robot_description"
                ],
                output="screen"
            )

            # Controller node (per robot)
            #behavior_node = Node(
            #    package="swarm_basics",
            #    executable="robot_supervisor_3_movements",
            #    name="robot_supervisor",
            #    namespace=ns,
            #    parameters=[
            #        {"spawn_x": x},
            #        {"spawn_y": y}
            #    ],
            #    output="screen"
            #)

            # Image processor — depth camera → zone detection (CLEAR/LEFT/RIGHT/CORNER)
            # NOTE: superseded by image_human_processor (real human detection);
            # nothing subscribes to its detected_zones topic, so it is disabled
            # to reduce log noise. Re-enable if the band detector is needed.
            # cpp_node = Node(
            #     package="leo_image",
            #     executable="image_processor",
            #     name="image_processor",
            #     namespace=ns,
            #     parameters=[{"enable_gui": False}],
            #     output="screen"
            # )

            # Human detector — pick backend via `detector` arg:
            #   depth = image_human_processor (blob heuristics, leo_image)
            #   yolo  = yolo_human_processor (YOLOv8n on RGB + depth ranging)
            # Both publish human_detected / human_close / human_distance / human_angle.
            if det == 'yolo':
                human_node = Node(
                    package="swarm_basics",
                    executable="yolo_human_processor",
                    name="yolo_human_processor",
                    namespace=ns,
                    output="screen"
                )
            else:
                human_node = Node(
                    package="leo_image",
                    executable="image_human_processor",
                    name="image_human_processor",
                    namespace=ns,
                    parameters=[{"enable_gui": False}],
                    output="screen"
                )

            bump_node = Node(
                package="swarm_basics",
                executable="bump_counter",
                name="bump_counter",
                namespace=ns,
                output="screen",
                remappings=[
                    ('contact', f"/world/{WORLD_NAME}/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"),
                ],
            )

            # Dynamic speed slowing near humans (scales controller cmd_vel)
            velocity_adaptor_node = Node(
                package="swarm_basics",
                executable="velocity_adaptor",
                name="velocity_adaptor",
                namespace=ns,
                output="screen",
            )

            # Robot-robot proximity perception (feeds IsRobotClose BT plugin)
            robot_proximity_node = Node(
                package="swarm_basics",
                executable="robot_proximity",
                name="robot_proximity",
                namespace=ns,
                output="screen",
            )

            # LLM high-level decision layer (feeds CheckLlmAction BT plugin).
            # Skipped when enable_llm=false -> no /ns/llm_action published ->
            # the BT's LLM path stays inactive and the reactive rules run.
            if llm_on:
                llm_planner_node = Node(
                    package="swarm_basics",
                    executable="llm_planner",
                    name="llm_planner",
                    namespace=ns,
                    output="screen",
                )
                nodes.append(llm_planner_node)

            # Odom-to-TF bridge: publishes odom -> base_footprint transform
            odom_tf_node = Node(
                package="swarm_basics",
                executable="odom_tf_publisher",
                name="odom_tf_publisher",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True
                }],
                output="screen",
            )

            # LiDAR scan frame fixer: rewrites Gazebo's mangled frame to {ns}/lidar_link
            lidar_republish_node = Node(
                package="swarm_basics",
                executable="lidar_republish",
                name="lidar_republish",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True
                }],
                output="screen",
            )

            # Depth → laser scan — kept for LLM/SCT use, LiDAR is now the primary scan for Nav2
            depth_to_scan = Node(
                package="swarm_basics",
                executable="depth_to_scan_custom",
                name="depth_to_scan",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True,
                }],
                output="screen",
            )

            # All per-robot nodes: LiDAR + RealSense + bump + odom + lidar_republish + depth_to_scan + human detector + velocity_adaptor + robot_proximity (+ llm_planner if enabled)
            nodes += [state_pub, spawn_node, human_node, bump_node, velocity_adaptor_node,
                      robot_proximity_node,
                      odom_tf_node, lidar_republish_node, depth_to_scan]

        return nodes

    return LaunchDescription([
        detector_arg,
        enable_llm_arg,
        plot_node,
        social_logger_node,
        OpaqueFunction(function=lambda context: create_all_robot_nodes(context, robots_to_spawn))
    ])