#!/usr/bin/env python3
"""LiDAR scan frame fixer.

Gazebo Fortress emits the gpu_lidar scan in a mangled frame
({ns}/{ns}/base_footprint/lidar) and ignores the URDF <frame_id> tag.
This node rewrites the frame_id to the clean URDF frame ({ns}/lidar_link)
so the TF tree stays clean: base_link -> lidar_link, no bridge needed.

Raw topic (mangled frame)  ->  clean topic ({ns}/lidar/scan_clean)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarRepublish(Node):
    def __init__(self):
        super().__init__('lidar_republish')

        self.ns = self.get_namespace().strip('/')
        self.clean_frame = f"{self.ns}/lidar_link"

        # Subscribe to the raw Gazebo scan (mangled frame_id)
        self.sub = self.create_subscription(
            LaserScan, 'lidar/scan', self.scan_cb, 10)

        # Republish with a clean frame on a distinct topic
        self.pub = self.create_publisher(LaserScan, 'lidar/scan_clean', 10)

        self.get_logger().info(
            f'LidarRepublish active: rewriting scan frame to {self.clean_frame}')

    def scan_cb(self, msg: LaserScan):
        msg.header.frame_id = self.clean_frame
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRepublish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
