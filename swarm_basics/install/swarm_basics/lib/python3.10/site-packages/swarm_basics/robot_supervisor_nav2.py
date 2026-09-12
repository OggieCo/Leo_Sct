import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotSupervisorNav2(Node):
    def __init__(self):
        super().__init__('robot_supervisor_nav2')

        # 1. Output Publisher directly to the robot's wheels / simulation motors
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 2. Input Subscriber catching path commands coming out of Nav2
        self.nav_sub = self.create_subscription(
            Twist, 
            'cmd_vel_nav', 
            self._publish_cmd, 
            10
        )
        
        # Optional: Keep tracking laser zone strings for your console visibility
        self.sub = self.create_subscription(String, 'detected_zones', self.zone_callback, 10)
        self.obstacle_zones = []
        
        self.get_logger().info("🚀 Nav2 Safety Pass-Through Supervisor Active!")

    def zone_callback(self, msg):
        self.obstacle_zones = [z.strip() for z in msg.data.split(',') if z.strip()]

    # 3. Transparent Pass-Through
    def _publish_cmd(self, twist: Twist):
        safe_twist = Twist()
        
        # Exact vector translation requested by your pipeline spec
        safe_twist.linear.x = float(twist.linear.x)
        safe_twist.linear.y = float(twist.linear.y)
        safe_twist.linear.z = float(twist.linear.z)
        safe_twist.angular.x = float(twist.angular.x)
        safe_twist.angular.y = float(twist.angular.y)
        safe_twist.angular.z = float(twist.angular.z)
        
        # Instantly ship the raw velocities straight to the wheels topic
        self.cmd_pub.publish(safe_twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisorNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()