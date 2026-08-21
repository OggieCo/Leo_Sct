WORLD_NAME = "human_world"

ROBOT_POSITIONS = [
    # 90-degree CROSSING test: robot_0 drives south along x=0,
    # robot_1 drives east along y=0 — their paths intersect at (0,0).
    #("robot_0",  0.0,  3.0, 4.7124),   # (0, 3)   facing -y -> drives south
    #("robot_1", -3.0,  0.0, 0.0),      # (-3, 0)  facing +x -> drives east

    # Opposite-standing (head-on) config: one vs one, face to face.
    ("robot_0",  0.25, 2.5, 4.7124),      # (0, 2.5)   facing -y (south)
    ("robot_1",  -0.25, -2.5, 1.5708),     # (0, -2.5)  facing +y (north)
]
