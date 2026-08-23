#!/usr/bin/env python3
"""velocity_adaptor — dynamic speed slowing near humans AND other robots.

Sits between Nav2's controller output and the robot driver:

    controller_server --cmd_vel_social--> velocity_adaptor --cmd_vel--> driver

It scales the commanded LINEAR velocity by a factor 0..1 AND scales the
angular velocity by the same factor (so the rover decelerates smoothly and
never pirouettes in place near people/robots), combining two independent
terms:

  * HUMAN term (YOLO human detection):
      - distance term — the closer the human, the slower (ramp d_slow..d_stop).
      - angle term    — strongest dead ahead, still noticeable from the side.
      - motion term   — fast bearing change (side approach) -> extra slowing.

  * ROBOT term (robot_proximity, nearest_robot_dist / robot_angle):
      - smooth deceleration as another rover approaches (ramp rd_slow..rd_stop)
        whether it is heading towards us or crossing our path;
      - strongest when the other rover is dead ahead (robot_angle term).

This gives a graceful slow-down before the BT's hard yield, so a head-on
robot encounter decelerates instead of hitting at full speed.

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

        # Robot-robot proximity (from robot_proximity, same namespace)
        self.declare_parameter('rd_stop', 1.0)        # m  -> scale 0 near another rover
        self.declare_parameter('rd_slow', 5.0)        # m  -> start slowing near another rover
        self.declare_parameter('lat_margin', 0.7)     # m  lateral gap counts as a safe pass

        # Nav2 controller now publishes to cmd_vel_social (see nav2_slam_all)
        self._sub_cmd = self.create_subscription(Twist, 'cmd_vel_social', self.cmd_cb, 10)
        self._pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._pub_scale = self.create_publisher(Float32, 'social_speed_scale', 10)

        self._sub_det = self.create_subscription(Bool, 'human_detected', self.det_cb, 10)
        self._sub_close = self.create_subscription(Bool, 'human_close', self.close_cb, 10)
        self._sub_dist = self.create_subscription(Float32, 'human_distance', self.dist_cb, 10)
        self._sub_ang = self.create_subscription(Float32, 'human_angle', self.ang_cb, 10)
        self._sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self._sub_rdist = self.create_subscription(Float32, 'nearest_robot_dist', self.rdist_cb, 10)
        self._sub_rang = self.create_subscription(Float32, 'robot_angle', self.rang_cb, 10)
        # LLM soft speed cap (llm_planner): 0..1, 1 = no limit from the AI
        self._llm_scale = 1.0
        self._sub_llm = self.create_subscription(Float32, 'llm_speed_scale', self.llm_cb, 10)

        self._detected = False
        self._close = False
        self._dist = float('inf')
        self._angle = 0.0
        self._yaw_rate = 0.0
        self._angle_rate = 0.0      # deg/s, human bearing rate minus robot yaw
        self._last_ang = None
        self._last_t = None

        self._robot_dist = float('inf')   # m, nearest other rover
        self._robot_angle = 0.0           # deg, bearing of that rover (+left)
        self._last_rdist = float('inf')   # distance-rate tracking (closing/separating)
        self._last_rdist_t = None
        self._rdist_rate = 0.0            # m/s, + = separating, - = approaching

        self.get_logger().info(
            f'VelocityAdaptor [{self.ns}]: slowing near humans '
            f'(d_slow={self.get_parameter("d_slow").value:.1f} m, '
            f'd_stop={self.get_parameter("d_stop").value:.1f} m, '
            f'rate_max={self.get_parameter("rate_max").value:.0f} deg/s) '
            f'+ robots (rd_slow={self.get_parameter("rd_slow").value:.1f} m, '
            f'rd_stop={self.get_parameter("rd_stop").value:.1f} m, '
            f'lat_margin={self.get_parameter("lat_margin").value:.2f} m)')

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

    def rdist_cb(self, msg):
        d = float(msg.data)
        self._robot_dist = d if math.isfinite(d) and d > 0.0 else float('inf')

    def rang_cb(self, msg):
        self._robot_angle = float(msg.data)

    def llm_cb(self, msg):
        self._llm_scale = max(0.0, min(1.0, float(msg.data)))

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

    def _robot_scale(self):
        """0..1 speed factor from proximity of the nearest other rover.

        Slows ONLY when the other rover is genuinely IN OUR WAY: it must be
        (a) ahead of us (front cone, |angle| <= 60 deg), (b) still CLOSING
        (distance decreasing), and (c) laterally close to our path
        (lat < lat_margin).  A rover passing side-by-side with a bigger
        lateral gap is NOT in our way -> no slowdown (they used to stall the
        whole pass).
        """
        d = self._robot_dist
        if not math.isfinite(d):
            return 1.0
        # closing rate (m/s): + = separating, - = approaching
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_rdist_t is not None:
            dt = now - self._last_rdist_t
            if dt > 1e-3:
                ddot = (d - self._last_rdist) / dt
                self._rdist_rate = 0.5 * self._rdist_rate + 0.5 * ddot
        self._last_rdist = d
        self._last_rdist_t = now

        # not in front (side / behind) -> no slowdown
        if abs(self._robot_angle) > 60.0:
            return 1.0
        # already passing / separating -> full speed
        if self._rdist_rate > 0.05:
            return 1.0

        # decompose the other rover's position relative to our heading into
        # "in our way" (lon, ahead) and "off to the side" (lat).
        a = math.radians(min(abs(self._robot_angle), 90.0))
        lat = d * math.sin(a)          # m off to the side (0 = dead ahead)
        lon = d * math.cos(a)          # m still ahead to close

        lat_margin = self.get_parameter('lat_margin').value  # safe-pass gap (m)
        # 1.0 = passes with a > lat_margin gap -> no slowdown
        # 0.0 = dead ahead -> full slowdown
        lat_term = min(max(lat / lat_margin, 0.0), 1.0)

        lon_stop = self.get_parameter('rd_stop').value   # m ahead -> scale 0
        lon_slow = self.get_parameter('rd_slow').value   # m ahead -> start slowing
        t = min(max((lon - lon_stop) / (lon_slow - lon_stop), 0.0), 1.0)
        lon_term = math.sqrt(t)        # sqrt -> smooth gradient deceleration

        # Complementary combination: a safe lateral pass (lat_term -> 1) keeps
        # FULL speed no matter how small lon is (the rover is beside us, NOT
        # in our way).  Only when BOTH lat and lon are small do we slow down.
        return 1.0 - (1.0 - lat_term) * (1.0 - lon_term)

    # -- output ------------------------------------------------------------
    def cmd_cb(self, msg):
        s = self._scale() * self._robot_scale() * self._llm_scale
        out = Twist()
        out.linear.x = msg.linear.x * s
        out.linear.y = msg.linear.y * s
        # Scale angular too: near another robot/human the rover must NOT
        # pirouette in place (adhesive wheels).  When the controller wants to
        # rotate while linear is suppressed (s -> 0), we drop the yaw rate as
        # well — no in-place spin.  When clear (s=1) turning is unaffected.
        # (Observed run_2026-08-23_17-56-11: post-arc re-orientation spun the
        #  rovers in place at cmd_lin=0 / cmd_ang=-1 for ~8 s.)
        out.angular.z = msg.angular.z * s
        self._pub_cmd.publish(out)
        fs = Float32()
        fs.data = float(s)
        self._pub_scale.publish(fs)
        if s < 0.99:
            self.get_logger().info(
                f'slowing: scale={s:.2f} human_dist={self._dist:.2f} '
                f'robot_dist={self._robot_dist:.2f} '
                f'robot_angle={self._robot_angle:+.1f} deg/s',
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
