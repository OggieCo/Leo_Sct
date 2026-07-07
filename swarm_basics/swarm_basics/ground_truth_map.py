#!/usr/bin/env python3
"""Publishes a static ground-truth OccupancyGrid of the corridor with cube."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class GroundTruthMap(Node):
    def __init__(self):
        super().__init__('ground_truth_map')

        # --- Corridor parameters ---
        resolution = 0.05          # 5 cm/pixel
        wall_y = 3.0               # y=3 (top wall), y=-3 (bottom wall)
        wall_x_min, wall_x_max = -10.0, 10.0
        wall_thickness = 0.2       # wall box size in y
        cube_half = 0.25           # half of 0.5m cube

        # --- Map bounds ---
        x_min, x_max = -12.5, 12.5
        y_min, y_max = -5.0, 5.0
        width = int((x_max - x_min) / resolution)
        height = int((y_max - y_min) / resolution)

        # --- Build occupancy grid ---
        grid = np.full((height, width), -1, dtype=np.int8)  # unknown

        # Fill free space (corridor interior)
        y_bottom = int((y_min + wall_thickness) / resolution)
        y_top = int((y_max - wall_thickness) / resolution)
        for j in range(height):
            y_world = y_min + (j + 0.5) * resolution
            if -wall_y + wall_thickness/2 < y_world < wall_y - wall_thickness/2:
                grid[j, :] = 0  # free

        # Fill walls
        for j in range(height):
            y_world = y_min + (j + 0.5) * resolution
            # Top wall (y ≈ 3)
            if abs(y_world - wall_y) < wall_thickness / 2:
                x_start = int((wall_x_min - x_min) / resolution)
                x_end = int((wall_x_max - x_min) / resolution)
                grid[j, x_start:x_end] = 100
            # Bottom wall (y ≈ -3)
            if abs(y_world + wall_y) < wall_thickness / 2:
                x_start = int((wall_x_min - x_min) / resolution)
                x_end = int((wall_x_max - x_min) / resolution)
                grid[j, x_start:x_end] = 100

        # Fill cube at (0, 0)
        cx_start = int((-cube_half - x_min) / resolution)
        cx_end = int((cube_half - x_min) / resolution)
        cy_start = int((-cube_half - y_min) / resolution)
        cy_end = int((cube_half - y_min) / resolution)
        grid[cy_start:cy_end, cx_start:cx_end] = 100

        # --- Publish ---
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, 'ground_truth_map', qos)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = x_min
        msg.info.origin.position.y = y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        # Publish once (latched via QOS=transient_local)
        self.pub.publish(msg)
        self.get_logger().info(f'Ground truth map published: {width}x{height}')


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
