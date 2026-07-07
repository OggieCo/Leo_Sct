#!/usr/bin/env python3
"""Simple depth image → laser scan converter without sync requirements."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
import math


class DepthToScan(Node):
    def __init__(self):
        super().__init__('depth_to_scan_custom')

        self.camera_info = None
        self.cam_info_sub = self.create_subscription(
            CameraInfo, 'depth_camera/camera_info', self.camera_info_cb, 10)

        self.depth_sub = self.create_subscription(
            Image, 'depth_camera/depth_image', self.depth_cb, 10)

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Static TF for depth_camera frame (Gazebo gives weird nested frame_id)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.get_logger().info('DepthToScan custom node started')

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def depth_cb(self, msg: Image):
        if self.camera_info is None:
            return  # Wait for first camera_info

        # Parse depth image (32FC1 = 32-bit float, 1 channel)
        raw = np.array(msg.data, dtype=np.uint8)
        depth = raw.view(np.float32).reshape(msg.height, msg.width)

        # --- FOV calculation ---
        hfov = 1.51844  # 87 degrees HFOV from URDF
        angles = np.linspace(-hfov / 2, hfov / 2, msg.width)

        # Build LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        ns = self.get_namespace()
        if ns and ns != '/':
            scan.header.frame_id = f"{ns.strip('/')}/realsense_link"

        scan.angle_min = float(angles[0])
        scan.angle_max = float(angles[-1])
        scan.angle_increment = float(angles[1] - angles[0])
        scan.time_increment = 0.0
        scan.scan_time = 0.2
        scan.range_min = 0.1
        scan.range_max = 10.0

        # --- Row selection ---
        # Camera is pitched down 17° (-0.3 rad).
        # VFOV = 2*atan(tan(87°/2)*48/64) ≈ 70.8°
        # With 17° downward pitch, horizontal plane is ~26% from top: row ~12
        # Sample rows 8-16 (covering horizon) and take min range per column
        vfov = 2.0 * np.arctan(np.tan(hfov / 2.0) * msg.height / msg.width)
        pitch = -0.3  # downward from URDF
        horizon_row = int((vfov / 2.0 - abs(pitch)) / vfov * msg.height)
        row_start = max(0, horizon_row - 4)
        row_end = min(msg.height, horizon_row + 5)

        ranges = np.min(depth[row_start:row_end, :], axis=0)

        # Replace infinity/NaN/too-close with max range
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        ranges[ranges < 0.01] = 10.0
        scan.ranges = ranges.tolist()
        scan.intensities = [0.0] * msg.width

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
