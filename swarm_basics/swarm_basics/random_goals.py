#!/usr/bin/env python3
"""Publishes random navigation goals for Nav2.
Robot explores randomly, builds a map via SLAM, avoids obstacles via costmap.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random
import math


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__('random_goal_publisher')

        # Accept ROS arg for robot namespace (e.g., robot_0)
        self.declare_parameter('robot_ns', 'robot_0')
        robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value

        action_name = f'/{robot_ns}/bt_navigator/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self.timer = self.create_timer(8.0, self.send_goal)
        self.get_logger().info(f'RandomGoalPublisher started — sending goals every 8s on {action_name}')

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not available')
            return

        # Random position in a 6x6m box around origin (within corridor world)
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        yaw = random.uniform(-math.pi, math.pi)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        self.get_logger().info(f'Sending goal: ({x:.1f}, {y:.1f})')
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down random goal publisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
