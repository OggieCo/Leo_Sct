# Shared robot configuration – edit this file to add/remove/change robots.
# Both spawn_multi_robots.launch.py and set_initial_pose.py read from here.
#
# Format: (namespace, x, y, yaw_in_radians)

ROBOT_POSITIONS = [
    ("robot_0", -6.5, 0.0, 0.0),
    # Uncomment and add more robots as needed:
    # ("robot_1",  6.5, 0.0, 3.14159),
    # ("robot_2",  0.0, 1.0, 0.0),
]
