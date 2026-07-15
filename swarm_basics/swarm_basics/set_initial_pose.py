import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from swarm_basics.robot_config import ROBOT_POSITIONS


class SetInitialPose(Node):
    """Publishes initial pose for every robot and waits for subscribers."""

    def __init__(self):
        super().__init__('set_initial_pose')
        self.get_logger().info('Setting initial poses from robot_config.py ...')

        # Publish one message per robot
        for ns, x, y, yaw in ROBOT_POSITIONS:
            if ns == 'robot_0':
                topic = '/initialpose'
            else:
                topic = f'/{ns}/initialpose'

            pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

            # Wait up to 10s for a subscriber (AMCL) to connect
            timeout = time.time() + 10.0
            while time.time() < timeout and pub.get_subscription_count() == 0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if pub.get_subscription_count() == 0:
                    self.get_logger().info(f'Waiting for subscriber on {topic}...')

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance = [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
            ]

            # Publish a few times to ensure delivery
            for i in range(3):
                msg.header.stamp = self.get_clock().now().to_msg()
                pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.2)

            self.get_logger().info(f'Published initial pose for {ns} ({x}, {y}) on {topic}')

        self.get_logger().info('All initial poses published. Done.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
