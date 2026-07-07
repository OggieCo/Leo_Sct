import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class OdomTfPublisher(Node):
    """Reads Odometry and publishes odom -> base_footprint transform to /tf."""

    def __init__(self):
        super().__init__('odom_tf_publisher')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_yaw = self.get_parameter('spawn_yaw').value

        # Publish directly to root /tf so RViz sees it (bypasses namespace)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)

        self.sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # use odom timestamp, not now()
        t.header.frame_id = msg.header.frame_id  # 'odom'
        # Use namespaced child frame: "robot_0/base_footprint" from odom message
        t.child_frame_id = msg.child_frame_id     # 'robot_0/base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x + self.spawn_x
        t.transform.translation.y = msg.pose.pose.position.y + self.spawn_y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        tf_msg = TFMessage()
        tf_msg.transforms.append(t)
        self.tf_pub.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
