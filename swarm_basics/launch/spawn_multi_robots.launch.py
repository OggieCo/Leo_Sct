import os
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Load robot positions from shared config (edit robot_config.py to add/remove robots)
from swarm_basics.robot_config import ROBOT_POSITIONS

def generate_launch_description():

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
 

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context, robots):
        nodes = []

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

                # ⬇ IN: Gazebo raw depth data → bridge (currently no subscriber, available for future use)
                f"/{ns}/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",

                # ⬇ IN: Gazebo camera calibration → bridge → image_processor (knows how to interpret pixels)
                f"/{ns}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",

                # ⬇ IN: Gazebo depth image → bridge → image_processor → detects obstacles (CLEAR/LEFT/RIGHT/CORNER)
                f"/{ns}/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",

                # ⬇ IN: Gazebo collision sensor → bridge → bump_counter logs "ouch, I hit something"
                f"/world/custom_corridor/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"
                f"@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",

                # ⬇ IN: Gazebo wheel positions → bridge → robot_state_publisher → TF → RViz shows spinning wheels
                f"/{ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ]

        # ⬇ IN: Gazebo publishes ALL model positions → bridge → coverage_plotter (tracks visited grid cells)
        bridge_args += [
            #"/world/u_corridor/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            #"/world/random_world/dynamic_pose/info@tf2_msgs/msg/TFMessage]ignition.msgs.Pose_V",
            "/world/custom_corridor/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
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

            cpp_node = Node(
                package="leo_image",
                executable="image_processor",
                name="image_processor",
                namespace=ns,
                parameters=[{"enable_gui": False}],  # Set True to show OpenCV debug windows
                output="screen"
            )

            bump_node = Node(
                package="swarm_basics",
                executable="bump_counter",
                name="bump_counter",
                namespace=ns,
                output="screen",
                remappings=[
                    ('contact', f"/world/custom_corridor/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"),
                ],
            )

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

            # Custom depth → fake laser scan (handles 87° FOV, camera pitch)
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

            nodes += [state_pub, spawn_node, cpp_node, bump_node, odom_tf_node, depth_to_scan]

        return nodes

    return LaunchDescription([
        plot_node,
        OpaqueFunction(function=lambda context: create_all_robot_nodes(context, robots_to_spawn))
    ])