import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
import random
import signal
import time


class RandomWalk(Node):
    """Wanders randomly, avoiding obstacles; SOCIALLY YIELDS (stops & waits)
    when a HUMAN is detected directly in front, then resumes and detours
    around it — maintaining its overall trajectory.

    Social vs reactive: the LiDAR swerves around ANY obstacle immediately
    (walls included), but the camera-based human detector (image_human_processor
    -> /robot_X/human_close + /robot_X/human_angle) makes the rover STOP and
    give way specifically for PEOPLE.
    """

    def __init__(self):
        super().__init__('random_walk')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # NOTE: the real LiDAR topic is /robot_X/lidar/scan_clean (lidar_republish
        # rewrites the mangled Gazebo frame); 'scan' does NOT exist and silently
        # disables ALL obstacle avoidance (rover drove into the human!).
        self.scan_sub = self.create_subscription(LaserScan, 'lidar/scan_clean', self.scan_callback, 10)
        self.human_close_sub = self.create_subscription(Bool, 'human_close', self.human_close_cb, 10)
        self.human_angle_sub = self.create_subscription(Float32, 'human_angle', self.human_angle_cb, 10)
        self.social_pub = self.create_publisher(String, 'social_message', 10)

        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        # Human detection state (from image_human_processor)
        self.human_close = False
        self.human_angle = 0.0       # deg, + = LEFT (image convention)

        # Movement parameters
        self.forward_speed = 0.2
        self.turn_speed = 0.8
        self.obstacle_threshold = 0.5  # meters – start turning if something closer
        self.turn_duration_min = 1.0   # seconds to turn at least
        self.turn_duration_max = 3.0   # seconds to turn at most
        self.turning = False
        self.turn_end_time = 0.0
        self.turn_direction = 1.0  # 1.0 = left, -1.0 = right

        # --- Social yield parameters (wait-and-see) ---
        self.declare_parameter('human_block_angle', 20.0)  # deg — "blocking" if |angle| < this
        self.human_block_angle = self.get_parameter('human_block_angle').value
        self.yield_wait_max = 7.0      # s — MAX time to stop & wait for the human to move
        self.resume_cooldown = 1.0     # s — brief guard before re-triggering after a resume
        self.avoid_turn_duration = 2.2 # s — turn AWAY from the human after yielding
        self.yield_cooldown = 3.0      # s — after avoiding, ignore re-trigger
        self.yielding = False
        self.yield_start = 0.0         # when the current wait began
        self.yield_end_time = 0.0
        self.avoiding = False
        self.avoid_end_time = 0.0
        self.avoid_direction = 1.0     # 1.0 = turn left (away from human), -1.0 = right
        self.cooldown_until = 0.0

        # Timer to keep publishing velocity commands
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'RandomWalk started — wait-and-see social nav: stop for humans, '
            f'proceed if they move (< {self.yield_wait_max:.0f} s), else detour')

    def scan_callback(self, msg: LaserScan):
        """Analyse laser scan to find closest obstacle in front, left, right."""
        n = len(msg.ranges)
        front = msg.ranges[n // 2 - 5:n // 2 + 5] if n > 10 else msg.ranges
        left = msg.ranges[:n // 4] if n > 4 else [float('inf')]
        right = msg.ranges[3 * n // 4:] if n > 4 else [float('inf')]

        self.front_min = min(front)
        self.left_min = min(left)
        self.right_min = min(right)

    def human_close_cb(self, msg: Bool):
        self.human_close = msg.data

    def human_angle_cb(self, msg: Float32):
        self.human_angle = msg.data

    def publish_social(self, text: str):
        m = String()
        m.data = text
        self.social_pub.publish(m)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()

        # --- YIELDING: stopped, WAITING for the human to move ---
        if self.yielding:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            blocking = (self.human_close
                        and abs(self.human_angle) < self.human_block_angle)
            if not blocking:
                # Human moved (away, or out of the blocking cone) BEFORE the
                # deadline → just proceed forward, NO detour.
                self.yielding = False
                self.cooldown_until = now + self.resume_cooldown
                self.get_logger().info(
                    f'Human moved after {now - self.yield_start:.1f} s '
                    f'(< {self.yield_wait_max:.0f} s) — proceeding forward')
                self.publish_social('thank you — continuing')
            elif now >= self.yield_end_time:
                # Human still blocking after the full wait → detour around, on
                # the side with MORE free space (LiDAR is fresh and reliable).
                self.yielding = False
                self.avoid_direction = 1.0 if self.left_min > self.right_min else -1.0
                self.avoiding = True
                self.avoid_end_time = now + self.avoid_turn_duration
                self.get_logger().info(
                    f'Still blocked after {self.yield_wait_max:.0f} s — '
                    f'avoiding to the {"left" if self.avoid_direction > 0 else "right"} now')
                self.publish_social('moving around you')
            self.cmd_pub.publish(twist)
            return

        # --- AVOIDING: turn AWAY from the human, arcing around, then resume ---
        if self.avoiding:
            twist.linear.x = 0.1          # small forward: arc around, not just spin
            twist.angular.z = self.avoid_direction * self.turn_speed
            if now >= self.avoid_end_time:
                self.avoiding = False
                self.cooldown_until = now + self.yield_cooldown
                self.get_logger().info('Avoided the human — resuming wander')
                self.publish_social('resuming my route')
            self.cmd_pub.publish(twist)
            return

        # --- SOCIAL CHECK: human directly blocking our path? ---
        if (not self.turning and now >= self.cooldown_until
                and self.human_close and abs(self.human_angle) < self.human_block_angle):
            self.yielding = True
            self.yield_start = now
            self.yield_end_time = now + self.yield_wait_max
            self.get_logger().info(
                f'HUMAN directly ahead ({self.human_angle:+.0f} deg) — stopping, '
                f'will wait up to {self.yield_wait_max:.0f} s for them to move')
            self.publish_social('human ahead — I will wait')
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            return

        # --- normal wander / obstacle avoidance (LiDAR) ---
        if self.turning:
            # Still turning – keep the turn command
            twist.angular.z = self.turn_direction * self.turn_speed
            if now >= self.turn_end_time:
                self.turning = False
                self.get_logger().info('Turn complete → moving forward')
        else:
            # Check if obstacle ahead
            if self.front_min < self.obstacle_threshold:
                # Decide turn direction: prefer the side with more free space
                if self.left_min > self.right_min:
                    self.turn_direction = 1.0   # turn left
                else:
                    self.turn_direction = -1.0  # turn right

                turn_duration = random.uniform(self.turn_duration_min, self.turn_duration_max)
                self.turn_end_time = now + turn_duration
                self.turning = True

                twist.angular.z = self.turn_direction * self.turn_speed
                self.get_logger().info(
                    f'Obstacle {self.front_min:.2f}m ahead → turning '
                    f'{"left" if self.turn_direction > 0 else "right"} for {turn_duration:.1f}s'
                )
            else:
                # Safe to go forward
                twist.linear.x = self.forward_speed

        self.cmd_pub.publish(twist)

    def hard_stop(self):
        """Flush zero velocity to Gazebo while the middleware is still alive.

        Called from the SIGINT handler (and again in main's finally) BEFORE
        rclpy.shutdown() — publishing inside destroy_node() is too late because
        rclpy's own SIGINT handler has already invalidated the context.
        """
        stop = Twist()
        for _ in range(8):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

    def destroy_node(self):
        # NOTE: do NOT publish here — the context is already invalid by the
        # time destroy_node runs. Hard-stop happens in main() pre-shutdown.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalk()

    def _on_sigint(sig, frame):
        # rclpy installs its own SIGINT handler that calls rclpy.shutdown() and
        # invalidates the context BEFORE we can publish a stop — that's why the
        # rover kept driving on Ctrl+C. Override it so the context stays alive
        # long enough to flush cmd_vel=0 to Gazebo, then unwind normally.
        try:
            node.hard_stop()
            node.get_logger().info('Stopping robot...')
        except Exception:
            pass
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Belt-and-suspenders: try to stop again if the context is still valid.
        try:
            node.hard_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
