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
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

# Stale-cap watchdog: if the LLM speed cap stays at ~0 longer than this while the
# controller is demanding motion, treat it as a stale stop/yield latch and release it.
LLM_ZERO_TIMEOUT_S = 8.0


class VelocityAdaptor(Node):
    def __init__(self):
        super().__init__('velocity_adaptor')
        self.ns = self.get_namespace().strip('/') or 'root'

        # TOLERANCE PASS. Humans still slow the rover smoothly, but only once they are
        # actually relevant: braking now starts at 1.8 m, the side term is much weaker
        # and a crossing human can no longer crush the command.
        self.declare_parameter('d_stop', 0.35)        # m  -> scale 0
        # 1.8 -> 1.2 m: braking now starts only when the person is genuinely near.
        self.declare_parameter('d_slow', 1.2)         # m  -> start slowing
        self.declare_parameter('rate_max', 90.0)      # deg/s -> min factor
        self.declare_parameter('rate_min_factor', 0.45)
        # Detection hold (s): keep the last human term after `human_detected` drops
        # instead of snapping back to 1.0 in one frame.
        self.declare_parameter('det_hold_s', 1.5)

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
        # Stale-cap watchdog state (see LLM_ZERO_TIMEOUT_S).
        self._llm_zero_since = None
        self._llm_zero_warned = False
        self._sub_llm = self.create_subscription(Float32, 'llm_speed_scale', self.llm_cb, 10)
        # Speed FLOOR (llm_planner): 0..1, 0 = no floor.
        self._floor = 0.0
        self._sub_floor = self.create_subscription(
            Float32, 'social_speed_floor', self.floor_cb, 10)

        self._detected = False
        self._close = False
        self._last_det_t = 0.0
        self._last_human_term = 1.0
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
        if msg.data:
            self._last_det_t = time.time()

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
        v = max(0.0, min(1.0, float(msg.data)))
        if v > 0.02:
            self._llm_zero_since = None
            self._llm_zero_warned = False
        elif self._llm_zero_since is None:
            self._llm_zero_since = time.time()
        self._llm_scale = v

    def floor_cb(self, msg):
        self._floor = max(0.0, min(1.0, float(msg.data)))

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
        # DETECTION HOLD: a dropped detection keeps the last human term for
        # det_hold_s — the rover does not accelerate back to full cruise just
        # because the detector lost a person it was passing.
        if not self._detected:
            if (time.time() - self._last_det_t) < \
                    self.get_parameter('det_hold_s').value:
                return self._last_human_term
            return 1.0
        d = self._dist
        if not math.isfinite(d):
            d = 0.35 if self._close else 1.2   # fallback: close -> near-stop
        d_stop = self.get_parameter('d_stop').value
        d_slow = self.get_parameter('d_slow').value
        # DISTANCE GATE. Beyond the slowdown range a human is not our conflict yet:
        # return full speed; the braking ramp below (and every other layer) applies
        # inside d_slow only.
        if d >= d_slow:
            self._last_human_term = 1.0
            return 1.0
        d_term = min(max((d - d_stop) / (d_slow - d_stop), 0.0), 1.0)
        # angle: 1.0 dead ahead, 0.65 at +/-90 deg (side) — a human merely
        # passing beside us must not cost half our speed (tolerance pass).
        a = min(abs(self._angle), 90.0)
        a_term = 0.65 + 0.35 * math.cos(math.radians(a))
        # motion: fast bearing change (side approach) -> extra slowing
        r_max = self.get_parameter('rate_max').value
        r_min = self.get_parameter('rate_min_factor').value
        r_term = min(max(1.0 - abs(self._angle_rate) / r_max, r_min), 1.0)
        term = max(0.0, min(1.0, d_term * a_term * r_term))
        self._last_human_term = term
        return term

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

        # Complementary combination: a safe lateral pass (lat_term -> 1) keeps FULL
        # speed no matter how small lon is (the rover is beside us, NOT in our way).
        return 1.0 - (1.0 - lat_term) * (1.0 - lon_term)

    # -- output ------------------------------------------------------------
    def cmd_cb(self, msg):
        # Human term first, then the robot term with the maneuver floor applied, then
        # the LLM cap.
        llm = self._llm_scale
        if (llm <= 0.02 and self._llm_zero_since is not None
                and abs(msg.linear.x) > 0.02
                and time.time() - self._llm_zero_since > LLM_ZERO_TIMEOUT_S):
            # STALE-CAP WATCHDOG: an LLM cap stuck at 0 while the base command moves
            # means a latched stop/yield - ignore the cap.
            llm = 1.0
            if not self._llm_zero_warned:
                self._llm_zero_warned = True
                self.get_logger().warn(
                    f'llm_speed_scale stuck at 0 for '
                    f'{time.time() - self._llm_zero_since:.0f}s while the '
                    f'controller demands motion — stale stop/yield latch, '
                    f'releasing (treated as 1.0).')
        s = self._scale() * max(self._robot_scale(), self._floor) * llm
        out = Twist()
        out.linear.x = msg.linear.x * s
        out.linear.y = msg.linear.y * s
        # Scale angular too: near another robot/human the rover must NOT pirouette in
        # place (adhesive wheels).
        out.angular.z = msg.angular.z * s
        self._pub_cmd.publish(out)
        fs = Float32()
        fs.data = float(s)
        self._pub_scale.publish(fs)
        if s < 0.99:
            self.get_logger().info(
                f'slowing: scale={s:.2f} human_dist={self._dist:.2f} '
                f'robot_dist={self._robot_dist:.2f} '
                f'robot_angle={self._robot_angle:+.1f} floor={self._floor:.2f} '
                f'llm={llm:.2f}',
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
