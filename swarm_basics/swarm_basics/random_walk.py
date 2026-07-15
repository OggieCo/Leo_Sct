import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random


class RandomWalk(Node):
    """Wanders around randomly, avoiding obstacles using laser scan data."""

    def __init__(self):
        super().__init__('random_walk')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        # Movement parameters
        self.forward_speed = 0.2
        self.turn_speed = 0.8
        self.obstacle_threshold = 0.5  # meters – start turning if something closer
        self.turn_duration_min = 1.0   # seconds to turn at least
        self.turn_duration_max = 3.0   # seconds to turn at most
        self.turning = False
        self.turn_end_time = 0.0
        self.turn_direction = 1.0  # 1.0 = left, -1.0 = right

        # Timer to keep publishing velocity commands
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RandomWalk started — exploring autonomously!')

    def scan_callback(self, msg: LaserScan):
        """Analyse laser scan to find closest obstacle in front, left, right."""
        n = len(msg.ranges)
        front = msg.ranges[n // 2 - 5:n // 2 + 5] if n > 10 else msg.ranges
        left = msg.ranges[:n // 4] if n > 4 else [float('inf')]
        right = msg.ranges[3 * n // 4:] if n > 4 else [float('inf')]

        self.front_min = min(front)
        self.left_min = min(left)
        self.right_min = min(right)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()

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

    def destroy_node(self):
        # Stop the robot on shutdown
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
