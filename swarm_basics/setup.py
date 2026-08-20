from setuptools import find_packages, setup

package_name = 'swarm_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_multi_robots.launch.py']),
        ('share/' + package_name + '/launch', ['launch/leo_gz.launch.py']),
        ('share/' + package_name + '/config/nav2', ['config/nav2/robot_0_nav2.yaml']),
        ('share/' + package_name + '/config/nav2', ['config/nav2/corridor_map.yaml']),
        ('share/' + package_name + '/config/nav2', ['config/nav2/corridor_map.pgm']),

        #slam toolbox config/launch files
        ('share/' + package_name + '/launch', ['launch/nav2_slam_all.launch.py']),
        #known-map mode (map_server + AMCL) for all robots
        ('share/' + package_name + '/launch', ['launch/nav2_map_all.launch.py']),
        ('share/' + package_name + '/launch', ['launch/random_walk_all.launch.py']),
        ('share/' + package_name + '/launch', ['launch/random_goals_all.launch.py']),
        ('share/' + package_name + '/launch', ['launch/slam_toolbox.launch.py']),
        ('share/' + package_name + '/config/nav2', ['config/nav2/slam_toolbox.yaml']),
        ('share/' + package_name + '/config/nav2', ['config/nav2/nav2_generic.yaml']),

        #behavior tree files (social_nav.xml is the active runtime tree)
        ('share/' + package_name + '/config/bt', ['config/bt/social_nav.xml']),
        ('share/' + package_name + '/config/bt', ['config/bt/navigate_to_pose_w_replanning_and_recovery.xml']),

        ('share/' + package_name + '/config', ['config/supervisor.yaml']),
        ('share/' + package_name + '/config', ['config/supervisor2.yaml']),
        ('share/' + package_name + '/config', ['config/sup_gpt.yaml']),
        ('share/' + package_name + '/config', ['config/cylinder_positions.json']),
        ('share/' + package_name + '/worlds', ['worlds/random_world.sdf']),
        ('share/' + package_name + '/worlds', ['worlds/human_world.sdf']),
        ('share/' + package_name + '/worlds', ['worlds/corridor_with_cube.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ecem',
    maintainer_email='ecem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'robot_supervisor_3_movements = swarm_basics.robot_supervisor_3_movements:main',
            'robot_supervisor_nav2 = swarm_basics.robot_supervisor_nav2:main',
            'coverage_plotter = swarm_basics.coverage_plotter:main',
            'bump_counter = swarm_basics.bump_counter:main',
            'odom_tf_publisher = swarm_basics.odom_tf_publisher:main',
            'lidar_republish = swarm_basics.lidar_republish:main',
            'depth_to_scan_custom = swarm_basics.depth_to_scan:main',
            'set_initial_pose = swarm_basics.set_initial_pose:main',
            'random_walk = swarm_basics.random_walk:main',
            'random_goals = swarm_basics.random_goals:main',
            'yolo_human_processor = swarm_basics.yolo_human_processor:main',
            'send_goal = swarm_basics.send_goal:main',
            'velocity_adaptor = swarm_basics.velocity_adaptor:main',
            'robot_proximity = swarm_basics.robot_proximity:main',
            'social_event_logger = swarm_basics.social_event_logger:main',
        ],
    },
)
