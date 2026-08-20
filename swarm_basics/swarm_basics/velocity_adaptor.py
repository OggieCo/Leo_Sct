#!/usr/bin/env python3
"""velocity_adaptor — dynamic speed slowing near humans.

Sits between Nav2's controller output and the robot driver:

    controller_server --cmd_vel_social--> velocity_adaptor --cmd_vel--> driver

It scales the commanded linear velocity by a factor 0..1 computed from the
YOLO human detection:
  * distance term — the closer the human, the slower (linear ramp d_slow..d_stop).
  * angle term    — strongest when the human is dead ahead, still noticeable
                    from the side (the RealSense sees roughly +/-60 deg).
  * motion term   — if the human's bearing is changing fast (approaching from
                    the side / walking past), slow down further.

The scale is published on `social_speed_scale` for logging/evaluation.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class VelocityAdaptor(Node):
    def __init__(self):
        super().__init__('velocity_adaptor')
        self.ns = self.get_namespace().strip('/') or 'root'

        self.declare_parameter('d_stop', 0.4)         # m  -> scale 0
        self.declare_parameter('d_slow', 3.5)         # m  -> start slowing (earlier)
        self.declare_parameter('rate_max', 60.0)      # deg/s -> min factor
        self.declare_parameter('rate_min_factor', 0.15)

        # Nav2 controller now publishes to cmd_vel_social (see nav2_slam_all)
        self._sub_cmd = self.create_subscription(Twist, 'cmd_vel_social', self.cmd_cb, 10)
        self._pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._pub_scale = self.create_publisher(Float32, 'social_speed_scale', 10)

        self._sub_det = self.create_subscription(Bool, 'human_detected', self.det_cb, 10)
        self._sub_close = self.create_subscription(Bool, 'human_close', self.close_cb, 10)
        self._sub_dist = self.create_subscription(Float32, 'human_distance', self.dist_cb, 10)
        self._sub_ang = self.create_subscription(Float32, 'human_angle', self.ang_cb, 10)
        self._sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self._detected = False
        self._close = False
        self._dist = float('inf')
        self._angle = 0.0
        self._yaw_rate = 0.0
        self._angle_rate = 0.0      # deg/s, human bearing rate minus robot yaw
        self._last_ang = None
        self._last_t = None

        self.get_logger().info(
            f'VelocityAdaptor [{self.ns}]: slowing near humans '
            f'(d_slow={self.get_parameter("d_slow").value:.1f} m, '
            f'd_stop={self.get_parameter("d_stop").value:.1f} m, '
            f'rate_max={self.get_parameter("rate_max").value:.0f} deg/s)')

    # -- inputs ------------------------------------------------------------
    def det_cb(self, msg):
        self._detected = msg.data

    def close_cb(self, msg):
        self._close = msg.data

    def dist_cb(self, msg):
        d = float(msg.data)
        self._dist = d if math.isfinite(d) and d > 0.0 else float('inf')

    def odom_cb(self, msg):
        self._yaw_rate = math.degrees(msg.twist.twist.angular.z)

    def ang_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        ang = float(msg.data)
        if self._last_ang is not None and self._last_t is not None:
            dt = now - self._last_t
            if dt > 1e-3:
                raw = ang - self._last_ang
                while raw > 180.0:
                    raw -= 360.0
                while raw < -180.0:
                    raw += 360.0
                rate = raw / dt - self._yaw_rate  # remove robot rotation
                alpha = 0.3
                self._angle_rate = (1.0 - alpha) * self._angle_rate + alpha * rate
        self._last_ang = ang
        self._last_t = now
        self._angle = ang

    # -- scale model -------------------------------------------------------
    def _scale(self):
        if not self._detected:
            return 1.0
        d = self._dist
        if not math.isfinite(d):
            d = 0.4 if self._close else 2.5  # fallback: close -> near-stop
        d_stop = self.get_parameter('d_stop').value
        d_slow = self.get_parameter('d_slow').value
        d_term = min(max((d - d_stop) / (d_slow - d_stop), 0.0), 1.0)
        # angle: 1.0 dead ahead, 0.5 at +/-90 deg (side)
        a = min(abs(self._angle), 90.0)
        a_term = 0.5 + 0.5 * math.cos(math.radians(a))
        # motion: fast bearing change (side approach) -> extra slowing
        r_max = self.get_parameter('rate_max').value
        r_min = self.get_parameter('rate_min_factor').value
        r_term = min(max(1.0 - abs(self._angle_rate) / r_max, r_min), 1.0)
        return max(0.0, min(1.0, d_term * a_term * r_term))

    # -- output ------------------------------------------------------------
    def cmd_cb(self, msg):
        s = self._scale()
        out = Twist()
        out.linear.x = msg.linear.x * s
        out.linear.y = msg.linear.y * s
        out.angular.z = msg.angular.z  # keep turning authority
        self._pub_cmd.publish(out)
        fs = Float32()
        fs.data = float(s)
        self._pub_scale.publish(fs)
        if s < 0.99:
            self.get_logger().info(
                f'slowing: scale={s:.2f} dist={self._dist:.2f} '
                f'angle={self._angle:+.1f} rate={self._angle_rate:+.1f} deg/s',
                throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityAdaptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
