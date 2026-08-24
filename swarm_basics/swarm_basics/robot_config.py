import math

WORLD_NAME = "human_world"

ROBOT_POSITIONS = [
    # Opposite-standing (HEAD-ON): robot_0 at (0,2.5) facing -y,
    # robot_1 at (0,-2.5) facing +y — they face each other 5 m apart.
    #("robot_0",  0, 2.5, 4.7124),
    #("robot_1",  0, -2.5, 1.5708),

    # 90-degree CROSSING test: robot_0 drives south along x=0,
    ("robot_0",  0.0,  3.0, 4.7124),   # (0, 3)   facing -y -> drives south
    ("robot_1", -3.0,  0.0, 0.0),      # (-3, 0)  facing +x -> drives east

    # 135-degree CROSSING (active): robot_0 drives south along x=0,
    # robot_1 drives northeast along y=x — headings differ by 135 deg,
    # paths intersect near the origin.
    #("robot_0",  0.0,  3.0, 4.7124),      # (0, 3)   facing -y  (south)
    #("robot_1", -2.12, -2.12, 0.7854),    # (-2.12,-2.12) facing +45 deg
]


def world_to_map(robot_name, x, y, yaw_world=0.0):
    """Convert a WORLD-coordinate goal to the robot's SLAM map frame.

    Every rover runs its OWN SLAM instance, which anchors {ns}/map at the
    rover's spawn pose (x0, y0, yaw0): map = R(-yaw0) * (world - spawn).  So
    the same world goal is a DIFFERENT map goal per rover.  This helper makes
    `send_goal X Y` (and random goals) mean the same WORLD point for every
    rover, matching the world/coverage frame.  Returns (map_x, map_y, map_yaw).
    """
    spawn = next(
        ((px, py, pyaw)
         for (n, px, py, pyaw) in ROBOT_POSITIONS if n == robot_name),
        None)
    if spawn is None:
        return x, y, yaw_world   # unknown rover -> pass through (map coords)
    x0, y0, yaw0 = spawn
    vx, vy = x - x0, y - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    mx = vx * c + vy * s
    my = -vx * s + vy * c
    myaw = (yaw_world - yaw0) % (2.0 * math.pi)   # wrap to (-pi, pi]
    if myaw > math.pi:
        myaw -= 2.0 * math.pi
    return mx, my, myaw
