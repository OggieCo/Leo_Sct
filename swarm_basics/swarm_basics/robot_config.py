WORLD_NAME = "human_world"

ROBOT_POSITIONS = [
    # Opposite-standing (HEAD-ON): robot_0 at (0,2.5) facing -y,
    # robot_1 at (0,-2.5) facing +y — they face each other 5 m apart.
    #("robot_0",  0, 2.5, 4.7124),
    #("robot_1",  0, -2.5, 1.5708),

    # 90-degree CROSSING test: robot_0 drives south along x=0,
    ("robot_0",  0.0,  3.0, 4.7124),   # (0, 3)   facing -y -> drives south
    #("robot_1", -3.0,  0.0, 0.0),      # (-3, 0)  facing +x -> drives east

    # 135-degree CROSSING (reference):
    #("robot_0",  0.0,  3.0, 4.7124),
    #("robot_1", -2.12, -2.12, 0.7854),
]
