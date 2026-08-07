# Shared robot configuration – edit this file to add/remove/change robots.
# Both spawn_multi_robots.launch.py and set_initial_pose.py read from here.
#
# Format: (namespace, x, y, yaw_in_radians)

# Gazebo world name 
# Must match the <world name="..."> in the loaded .sdf file.
# Currently the random world WITH stationary humans (obstacles + humans).
# Revert to "human_world" for the walking-human social demo.
WORLD_NAME = "random_world_humans"

ROBOT_POSITIONS = [
    ("robot_0",  0.0, 0.0, 0.0),
    # ("robot_1",  1.0, 0.0, 0.0),
    # ("robot_2",  0.0, 1.0, 1.57),
    # ("robot_3", -1.0, 1.0, 3.14),
    # ("robot_4", -1.0, 0.0, -1.57),
]
