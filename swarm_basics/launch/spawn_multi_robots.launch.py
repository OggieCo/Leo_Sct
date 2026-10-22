import os
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    # bt: 'reactive' (reactive_nav.xml) / 'ai' (social_nav.xml) / 'stock' (stock
    # Nav2 tree; 'no' is accepted as a legacy alias).  Only used for run-folder
    # routing here ('stock' -> logs go to Nav2/); the actual behavior tree is
    # chosen in nav2_slam_all.launch.py.
    bt_arg = DeclareLaunchArgument(
        'bt', default_value='reactive',
        description='Behavior layer: reactive, ai or stock (run-folder routing)')

    # dynamic_plots: 'true' -> live coverage-map redraws every 0.5 s (PNGs every
    # 2 s); 'false' -> render the coverage map only once at shutdown (lighter
    # during the run; trajectories/events CSVs are still written continuously).
    dynamic_plots_arg = DeclareLaunchArgument(
        'dynamic_plots', default_value='true',
        description='Live coverage-map rendering (false = render only at shutdown)')

    # log_raw_contacts: 'true' -> ALSO write every raw contact-sensor message to
    # contacts_raw.csv (chatty, ~90 rows per second while touching).  Default
    # false: events.csv logs one CONTACT_BEGIN/CONTACT_END pair per episode.
    log_raw_contacts_arg = DeclareLaunchArgument(
        'log_raw_contacts', default_value='false',
        description='Also write every raw contact message to contacts_raw.csv')

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
            parameters=[{
                'dynamic_plots': ParameterValue(
                    LaunchConfiguration('dynamic_plots'), value_type=bool),
                'log_raw_contacts': ParameterValue(
                    LaunchConfiguration('log_raw_contacts'), value_type=bool),
            }],
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
    bt = LaunchConfiguration('bt')

    # Route run logs into a MODE subfolder:
    #   bt:=stock     -> Nav2/    (stock Nav2, no social layer)
    #   enable_llm:=true -> BT_LLM/
    #   else             -> BT/
    # run_utils.get_run_dir() reads SWARM_RUN_MODE so coverage_plotter,
    # social_event_logger and llm_planner all write into the same subfolder.
    run_mode_env = SetEnvironmentVariable(
        'SWARM_RUN_MODE',
        PythonExpression([
            "'Nav2' if '", bt, "'.lower() in ('no', 'stock') else ",
            "('BT_LLM' if '", enable_llm, "'.lower() == 'true' else 'BT')",
        ]),
    )

    # --- Function to create all robot nodes ---
    def create_all_robot_nodes(context, robots):
        nodes = []
        det = context.perform_substitution(detector)   # 'depth' or 'yolo'
        llm_on = context.perform_substitution(enable_llm).lower() in ('true', '1')
        bt_val = context.perform_substitution(bt).lower()   # reactive / ai / stock
        pure = (bt_val in ('no', 'stock'))   # stock Nav2: no human-awareness nodes

        # --- Bridges: ONE small global bridge + ONE PER ROBOT ---
        # (2026-09-18 load fix) A single parameter_bridge carried all ~46
        # channels (incl. 15 camera streams) in ONE event loop; under 5-robot
        # load that serialized the sensor pipeline — scans reached SLAM a
        # minute+ late and were dropped ("queue is full"), no map was
        # published and the boot gate could never pass.  Split per robot so
        # message conversion runs in parallel.  /tf is GLOBAL (Gazebo
        # broadcasts all model poses on one topic) — bridge it exactly once
        # here; it used to be added 5 times (once per robot).
        nodes.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="global_bridge",
            arguments=[
                # ⬇ IN: Gazebo broadcasts all model positions → /tf → RViz draws the world
                "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                # ⬇ IN: Global pose topic for coverage_plotter
                f"/world/{WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            ],
            parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
            output="screen",
        ))

        for bridge_i, robot in enumerate(robots):
            ns = robot["ns"]
            # Stagger (2026-09-18): robot i's bridge starts 4 s per index apart.
            # With all 5 robots' RSPs/spawners/bridges/sensors firing in the
            # same second, FastDDS SHM port files collided and spawners/managers
            # starved (robot_description never reached 2 of 5 spawners).
            nodes.append(TimerAction(
                period=4.0 * bridge_i,
                actions=[Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{ns}_bridge",
                arguments=[
                    # ⬆ OUT: supervisor sends motion → bridge feeds it to Gazebo → robot drives
                    f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                    # ⬇ IN: Gazebo tells where robot is → bridge → odom_tf_publisher & RViz read it
                    f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    # ⬇ IN: Gazebo LiDAR scan (360°) → bridge → Nav2 SLAM & costmap
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
                ],
                parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
                output="screen",
                )],
            ))

        # --- Create each robot (staggered 4 s per index, with its bridge) ---
        for robot_i, robot in enumerate(robots):
            ns = robot["ns"]
            x = robot["x"]
            y = robot["y"]
            yaw = robot["yaw"]

            # URDF with per-robot namespace mapping.  Sensor rates come from the
            # environment so a run can override them; the defaults match the
            # frozen data-collection configuration (5 Hz LiDAR / 3 Hz camera at
            # 320x240, SWARM_LIDAR_HZ / SWARM_CAMERA_HZ / SWARM_CAMERA_W/H).
            xacro_file = os.path.join(leo_description, 'urdf', 'leo_sim.urdf.xacro')
            doc = xacro.process_file(xacro_file, mappings={
                "robot_ns": ns,
                "lidar_rate": os.environ.get("SWARM_LIDAR_HZ", "5"),
                "camera_rate": os.environ.get("SWARM_CAMERA_HZ", "3"),
                "camera_width": os.environ.get("SWARM_CAMERA_W", "320"),
                "camera_height": os.environ.get("SWARM_CAMERA_H", "240"),
            })
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

            # Spawn robot in Gazebo.
            # 2026-09-18: spawn from the URDF STRING directly (same XML the RSP
            # publishes) instead of via the latched /robot_description topic.
            # The latched DDS delivery intermittently failed (spawner waited
            # forever, robot never appeared — hit both headless and GUI boots),
            # which stalled the whole bringup.  The string path has no DDS
            # dependency; `create` retries the factory service until the
            # gz server is up.
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
                    "-string", robot_description
                ],
                output="screen"
            )

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
            # Skipped in pure Nav2 (bt:=stock): no social layer consumes human data.
            if pure:
                human_node = None
            elif det == 'yolo':
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

            # Dynamic speed slowing near humans/robots (scales controller
            # cmd_vel). Disabled in pure Nav2 (bt:=stock): the stock Nav2 baseline
            # must have NO human/robot awareness, so the controller writes
            # /cmd_vel directly (see nav2_slam_all.launch.py) and rovers only
            # avoid what the lidar costmap sees.
            if pure:
                velocity_adaptor_node = None
            else:
                velocity_adaptor_node = Node(
                    package="swarm_basics",
                    executable="velocity_adaptor",
                    name="velocity_adaptor",
                    namespace=ns,
                    output="screen",
                )

            # Robot-robot proximity perception (feeds IsRobotClose BT plugin + velocity_adaptor)
            if pure:
                robot_proximity_node = None
            else:
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

            # Depth → laser scan.  VESTIGIAL as of 2026-09-23: both costmaps
            # (obstacle_layer.scan.topic = /{ns}/lidar/scan_clean) and SLAM
            # (remap 'scan' -> /{ns}/lidar/scan_clean) use the LiDAR scan, and
            # nothing subscribes to the depth-derived /{ns}/scan.  Producing it
            # costs one Python node per rover (depth image -> LaserScan), so it
            # is skipped by default; export SWARM_NO_DEPTH_TO_SCAN=0 to keep it.
            if os.environ.get("SWARM_NO_DEPTH_TO_SCAN", "1") == "0":
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
            else:
                depth_to_scan = None

            # All per-robot nodes EXCEPT the human detector (deferred below):
            # LiDAR + RealSense + bump + odom + lidar_republish + depth_to_scan
            # + velocity_adaptor + robot_proximity (+ llm_planner if enabled).
            # Pure Nav2 (bt:=stock) drops the human awareness nodes entirely.
            per_robot = [state_pub, spawn_node, bump_node,
                         velocity_adaptor_node, robot_proximity_node,
                         odom_tf_node, lidar_republish_node, depth_to_scan]
            # Stagger per robot (same 4 s/index cadence as its bridge) so the
            # RSP + spawn + sensor-node startup burst is spread over time.
            nodes.append(TimerAction(
                period=4.0 * robot_i,
                actions=[n for n in per_robot if n is not None],
            ))

            # Human detector, DELAYED by SWARM_DETECTOR_DELAY_S seconds
            # (default 45 s; 0 = old behaviour).  It costs a torch/YOLO model
            # load, a CUDA context and
            # one more DDS participant per robot, and consumes nothing until
            # goals start — so keeping it OUT of the bring-up window matters:
            # with 5 rovers that window already carries ~45 DDS participants,
            # and discovery starvation there is exactly what leaves lifecycle
            # managers stuck on "Waiting for service .../get_state".
            if human_node is not None:
                det_delay = float(
                    os.environ.get("SWARM_DETECTOR_DELAY_S", "0") or 0.0)
                # period = 4 s per robot + the delay.  The delay must clear the
                # WHOLE Nav2 stagger (nav2_slam_all.launch.py: 20 s + 12 s per
                # robot, so the last stack starts at 68 s and finishes ~100 s):
                # loading 5 YOLO models + CUDA contexts + DDS participants
                # inside that window is what aborted bring-up on 2026-09-23.
                nodes.append(TimerAction(
                    period=4.0 * robot_i + det_delay,
                    actions=[human_node],
                ))

        return nodes

    return LaunchDescription([
        detector_arg,
        enable_llm_arg,
        bt_arg,
        dynamic_plots_arg,
        log_raw_contacts_arg,
        run_mode_env,
        plot_node,
        social_logger_node,
        OpaqueFunction(function=lambda context: create_all_robot_nodes(context, robots_to_spawn))
    ])