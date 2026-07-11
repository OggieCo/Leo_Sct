import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from swarm_basics.robot_config import ROBOT_POSITIONS


class SetInitialPose(Node):
    """One-shot node that publishes initial pose for every robot in robot_config.py."""

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
            rclpy.spin_once(self, timeout_sec=0.3)   # let the publisher connect

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

            pub.publish(msg)
            self.get_logger().info(f'Published initial pose for {ns} ({x}, {y}) on {topic}')

        # Give DDS time to deliver the messages before shutting down
        time.sleep(1.0)
        self.get_logger().info('All initial poses published. Done.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    # Keep spinning while the node works
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
