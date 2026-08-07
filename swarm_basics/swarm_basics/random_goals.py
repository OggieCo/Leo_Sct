#!/usr/bin/env python3
"""Publishes random navigation goals for Nav2.
Robot explores randomly, builds a map via SLAM, avoids obstacles via costmap.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
import random
import math
import signal
import time


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__('random_goal_publisher')

        # Accept ROS arg for robot namespace (e.g., robot_0)
        self.declare_parameter('robot_ns', 'robot_0')
        robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.robot_ns = robot_ns

        # Action server is created relative to the NAMESPACE in Nav2 Humble,
        # NOT the node name — i.e. /robot_0/navigate_to_pose, not
        # /robot_0/bt_navigator/navigate_to_pose (the latter returns no server).
        action_name = f'/{robot_ns}/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self.timer = self.create_timer(8.0, self.send_goal)

        # Hard-stop publisher (killswitch): directly commands zero velocity so
        # the rover stops even if the goal cancel is slow.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._goal_handle = None
        self.get_logger().info(f'RandomGoalPublisher started — sending goals every 8s on {action_name}')

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not available')
            return

        # Random position in a 6x6m box around origin (within corridor world)
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        yaw = random.uniform(-math.pi, math.pi)

        # Per-robot map frame (each rover has its own {ns}/map tree)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = f'{self.robot_ns}/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        self.get_logger().info(f'Sending goal: ({x:.1f}, {y:.1f})')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        self._goal_handle = goal_handle

    def killswitch(self):
        """Stop the rover on shutdown: cancel the active Nav2 goal and flush a
        hard zero-velocity command to Gazebo BEFORE rclpy tears down the
        context (publishing in destroy_node is too late)."""
        self.get_logger().info('Stopping robot...')
        # 1) cancel the active Nav2 goal so the controller stops commanding
        try:
            if self._goal_handle is not None and self._goal_handle.is_active:
                cancel_future = self._goal_handle.cancel_goal_async()
                # give the cancel a moment to reach Nav2
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.05)
        except Exception:
            pass
        # 2) belt-and-suspenders: publish zero velocity directly
        stop = Twist()
        for _ in range(8):
            try:
                self.cmd_pub.publish(stop)
            except Exception:
                break
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalPublisher()

    def _on_sigint(sig, frame):
        # Override rclpy's SIGINT handler (which shuts down the context before
        # we can publish) so we can cancel the goal + flush a stop first.
        try:
            node.killswitch()
        except Exception:
            pass
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down random goal publisher.')
    finally:
        try:
            node.killswitch()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
