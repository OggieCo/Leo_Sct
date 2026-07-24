import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Explicitly declare namespace to capture frames correctly
        self.ns = self.get_namespace().strip('/')
        self.odom_frame = f"{self.ns}/odom" if self.ns else "odom"
        self.base_frame = f"{self.ns}/base_footprint" if self.ns else "base_footprint"
        self.map_frame = "map"
        
        # Subscribe to namespaced odometry data
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info(f"Odom-to-TF Bridge active: Broadcasting {self.odom_frame} -> {self.base_frame}")

    def odom_callback(self, msg):
        t = TransformStamped()

        # Enforce exact simulation timestamp
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Copy position vectors from Gazebo topic data
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy rotation quaternions
        t.transform.rotation = msg.pose.pose.orientation

        # Publish transform to global tree
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()