#!/usr/bin/env python3
"""send_goal — send a Nav2 NavigateToPose goal to a robot.

Usage:
  ros2 run swarm_basics send_goal X Y [--robot robot_0] [--yaw 0.0]

X Y --yaw are WORLD coordinates (the shared Gazebo/world frame, same as the
coverage map).  They are converted to the robot's own SLAM map frame before
sending, so the same goal means the same world point for every robot.
Ideal for the social-nav test: point the goal PAST the human (e.g. human at
(3,0) -> goal (5,0)) so the rover must pass near them and the BT yield/arc
kicks in.
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from swarm_basics.robot_config import world_to_map


class GoalSender(Node):
    def __init__(self, ns, x, y, yaw):
        super().__init__('send_goal')
        self.ns = ns
        # X Y --yaw are WORLD coordinates; convert to this rover's own SLAM
        # map frame (each rover's map is anchored at its own spawn pose).
        self.mx, self.my, self.myaw = world_to_map(ns, x, y, yaw)
        self.action = ActionClient(self, NavigateToPose, f'{ns}/navigate_to_pose')
        if (self.mx, self.my, self.myaw) != (x, y, yaw):
            self.get_logger().info(
                f'{ns}: world goal ({x}, {y}, yaw {yaw:.2f}) -> '
                f'map goal ({self.mx:.2f}, {self.my:.2f}, yaw {self.myaw:.2f})')

    def send(self):
        if not self.action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'No action server at {self.ns}/navigate_to_pose')
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = f'{self.ns}/map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.mx
        goal.pose.pose.position.y = self.my
        goal.pose.pose.orientation.z = math.sin(self.myaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.myaw / 2.0)
        self.get_logger().info(
            f'Sending goal to {self.ns}: map ({self.mx:.2f}, {self.my:.2f}, '
            f'yaw {self.myaw:.2f} rad)')
        self._goal_handle = self.action.send_goal_async(
            goal, feedback_callback=self.fb_cb)
        self._goal_handle.add_done_callback(self.result_cb)
        return True

    def fb_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'  feedback: dist_to_goal {fb.distance_remaining:.2f} m',
            throttle_duration_sec=2.0)

    def result_cb(self, future):
        handle = future.result()
        if handle is None:
            self.get_logger().error('Goal was rejected by bt_navigator')
            rclpy.shutdown()
            return
        result = handle.get_result()
        self.get_logger().info(
            f'Goal finished: status={handle.status}, result={result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    p = argparse.ArgumentParser(description='Send a Nav2 NavigateToPose goal')
    p.add_argument('x', type=float, help='goal X in WORLD coordinates')
    p.add_argument('y', type=float, help='goal Y in WORLD coordinates')
    p.add_argument('--robot', default='robot_0', help='robot namespace')
    p.add_argument('--yaw', type=float, default=0.0, help='goal yaw (rad, world)')
    a = p.parse_args()
    node = GoalSender(a.robot, a.x, a.y, a.yaw)
    if node.send():
        rclpy.spin(node)
    else:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
