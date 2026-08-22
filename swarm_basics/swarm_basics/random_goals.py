#!/usr/bin/env python3
"""Publishes random navigation goals for Nav2 for 1..N robots.

One node manages every robot in ROBOT_POSITIONS (robot_config.py).  Each robot
gets a new random goal ONLY when it has reached its own previous goal
(STATUS_SUCCEEDED), so robots explore independently without goal spam.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
import random
import math
import signal
import time

from swarm_basics.robot_config import ROBOT_POSITIONS


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__('random_goal_publisher')

        # Per-robot state: action client, stop publisher, goal handle, busy flag.
        self.robots = {}
        for ns, _, _, _ in ROBOT_POSITIONS:
            self.robots[ns] = {
                'action_client': ActionClient(
                    self, NavigateToPose, f'/{ns}/navigate_to_pose'),
                'cmd_pub': self.create_publisher(Twist, f'/{ns}/cmd_vel', 10),
                'goal_handle': None,
                'busy': False,
            }

        self.get_logger().info(
            f'RandomGoalPublisher started for {len(self.robots)} robot(s): '
            f"{', '.join(self.robots)} — a new goal is sent to each robot only "
            f'when it reaches its own')

        # Kick off the first goal for each robot once the node has spun up.
        for ns in self.robots:
            self._schedule(2.0, lambda ns=ns: self.send_goal(ns))

    def _schedule(self, delay, cb):
        """Run `cb()` once after `delay` seconds (self-cancelling timer)."""
        timer = self.create_timer(delay, lambda: self._fire_once(timer, cb))

    def _fire_once(self, timer, cb):
        timer.cancel()
        cb()

    def send_goal(self, ns):
        robot = self.robots[ns]
        if robot['busy']:
            return
        ac = robot['action_client']
        if not ac.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                f'[{ns}] Nav2 action server not available — retrying in 5s')
            self._schedule(5.0, lambda: self.send_goal(ns))
            return

        # Random position in a 6x6m box around origin (within corridor world)
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        yaw = random.uniform(-math.pi, math.pi)

        # Per-robot map frame (each rover has its own {ns}/map tree).
        # NO leading slash: Nav2's costmap global_frame is '{ns}/map' and
        # transformPoseInTargetFrame does a string compare — '/robot_0/map'
        # != 'robot_0/map' sent it down a TF lookup that fails.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = f'{ns}/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        robot['busy'] = True
        self.get_logger().info(f'[{ns}] Sending goal: ({x:.1f}, {y:.1f})')
        future = ac.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, ns=ns: self._goal_response_cb(f, ns))

    def _goal_response_cb(self, future, ns):
        robot = self.robots[ns]
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{ns}] Goal rejected by Nav2 — retrying in 5s')
            robot['busy'] = False
            self._schedule(5.0, lambda: self.send_goal(ns))
            return
        robot['goal_handle'] = goal_handle
        # Wait for the action result — only when THIS robot's goal SUCCEEDS do
        # we send it the next random goal.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, ns=ns: self._result_cb(f, ns))

    def _result_cb(self, future, ns):
        robot = self.robots[ns]
        result = future.result()
        robot['busy'] = False
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{ns}] Goal reached — sending a new random goal')
            self._schedule(0.5, lambda: self.send_goal(ns))
        else:
            self.get_logger().warn(
                f'[{ns}] Goal finished with status {status} (not SUCCEEDED) — '
                f'retrying in 5s')
            self._schedule(5.0, lambda: self.send_goal(ns))

    def killswitch(self):
        """Stop every rover on shutdown: cancel active Nav2 goals and flush
        zero-velocity commands to Gazebo BEFORE rclpy tears down the context
        (publishing in destroy_node is too late)."""
        self.get_logger().info('Stopping robot(s)...')
        for ns, robot in self.robots.items():
            # 1) cancel the active Nav2 goal so the controller stops commanding
            try:
                goal_handle = robot['goal_handle']
                if goal_handle is not None and goal_handle.is_active:
                    goal_handle.cancel_goal_async()
                    # give the cancel a moment to reach Nav2
                    for _ in range(10):
                        rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                pass
            # 2) belt-and-suspenders: publish zero velocity directly
            stop = Twist()
            for _ in range(8):
                try:
                    robot['cmd_pub'].publish(stop)
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
