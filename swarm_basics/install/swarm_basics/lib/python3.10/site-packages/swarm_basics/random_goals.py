#!/usr/bin/env python3
"""Publishes random navigation goals for Nav2 for 1..N robots.

One node manages every robot in ROBOT_POSITIONS (robot_config.py).  Each robot
gets a new random goal ONLY when it has reached its own previous goal
(STATUS_SUCCEEDED), so robots explore independently without goal spam.

Phased goal sampling: while a rover's own SLAM map is not yet scanned enough
(known-fraction < SCAN_THRESHOLD), goals are kept inside its field of view
(forward of its current heading) so it never gets a blind out-of-sight goal.
Once the map is >= SCAN_THRESHOLD known, goals span the full arena including
the +/-6..7 edge bands next to the walls.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import random
import math
import signal
import time

from swarm_basics.robot_config import ROBOT_POSITIONS, world_to_map


class RandomGoalPublisher(Node):
    # ---- Phased goal sampling ----
    # Phase 1 (map not scanned enough): goals stay in the rover's own field of
    # view — forward of its current heading — so it never gets a blind
    # out-of-sight goal (the robot_1 bug in the 14:56 run: a goal behind its
    # heading, into an unscanned area).  Phase 2 (map known-fraction >=
    # SCAN_THRESHOLD): full arena including the +/-6..7 edge bands.
    SCAN_THRESHOLD = 0.6     # known fraction of /{ns}/map that unlocks Phase 2
    ARENA_HALF = 6.5         # full-arena goal bounds (walls at +/-7.25)
    ARENA_SAFE = 6.0         # phase-1 clamp so FOV goals can't hit a wall
    FOV_DIST = (1.5, 4.5)    # forward-goal distance range (m)
    FOV_CONE_DEG = 60.0      # forward cone half-angle (deg)
    FOV_FILL_PROB = 0.25     # chance of a short any-direction fill goal
    FILL_DIST = (1.5, 3.5)   # fill-goal distance range (m)

    def __init__(self):
        super().__init__('random_goal_publisher')

        # Per-robot state: action client, stop publisher, goal handle, busy
        # flag, current odom pose, and SLAM map known-fraction (phased goals).
        self.spawns = {ns: (x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS}
        self.robots = {}
        for ns, _, _, _ in ROBOT_POSITIONS:
            self.robots[ns] = {
                'action_client': ActionClient(
                    self, NavigateToPose, f'/{ns}/navigate_to_pose'),
                'cmd_pub': self.create_publisher(Twist, f'/{ns}/cmd_vel', 10),
                'goal_handle': None,
                'busy': False,
                'pose': None,      # (x, y, yaw) in {ns}/odom frame
                'map_known': 0.0,  # known fraction of /{ns}/map
                'phase2': False,   # True once full-arena goals are unlocked
            }
            # SLAM occupancy grid -> "have we scanned enough of the arena?"
            self.create_subscription(
                OccupancyGrid, f'/{ns}/map',
                lambda msg, ns=ns: self._map_cb(msg, ns), 10)
            # Odom -> where is the rover right now (for forward-FOV goals)
            self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda msg, ns=ns: self._odom_cb(msg, ns), 10)

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

        # Phased goal — WORLD coordinates, then converted to this rover's own
        # SLAM map frame (each rover's map is anchored at its own spawn pose),
        # so all rovers explore the same world box.
        x, y, yaw = self._sample_goal(ns)
        mx, my, myaw = world_to_map(ns, x, y, yaw)

        # Per-robot map frame (each rover has its own {ns}/map tree).
        # NO leading slash: Nav2's costmap global_frame is '{ns}/map' and
        # transformPoseInTargetFrame does a string compare — '/robot_0/map'
        # != 'robot_0/map' sent it down a TF lookup that fails.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = f'{ns}/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = mx
        goal_msg.pose.pose.position.y = my
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = math.cos(myaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(myaw / 2.0)

        robot['busy'] = True
        self.get_logger().info(
            f'[{ns}] Sending goal: world ({x:.1f}, {y:.1f}) -> '
            f'map ({mx:.1f}, {my:.1f})')
        future = ac.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, ns=ns: self._goal_response_cb(f, ns))

    # ------------------------------------------------------------------ #
    # Phased goal sampling
    # ------------------------------------------------------------------ #
    def _map_cb(self, msg, ns):
        """Track the known (scanned) fraction of this rover's SLAM map."""
        robot = self.robots[ns]
        if not msg.data:
            return
        known = sum(1 for v in msg.data if v >= 0)
        frac = known / len(msg.data)
        robot['map_known'] = frac
        if not robot['phase2'] and frac >= self.SCAN_THRESHOLD:
            robot['phase2'] = True
            self.get_logger().info(
                f'[{ns}] map {frac*100:.0f}% known — unlocked full-arena '
                f'(edge) goals')

    def _odom_cb(self, msg, ns):
        """Remember the rover's current pose in its odom frame."""
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robots[ns]['pose'] = (p.position.x, p.position.y, yaw)

    def _sample_goal(self, ns):
        """Pick the next world goal (x, y, yaw) for robot `ns`.

        Phase 1 — map not scanned enough yet: a goal the rover can actually
        see.  Mostly a forward cone (within +/-FOV_CONE_DEG of its heading,
        FOV_DIST away); sometimes a short any-direction goal so it also fills
        the blind spot behind it.  Clamped to +/-ARENA_SAFE so a forward goal
        near the wall can't overshoot into it.

        Phase 2 — known-fraction of /{ns}/map >= SCAN_THRESHOLD: the whole
        arena, including the +/-6..7 edge bands next to the walls.
        """
        robot = self.robots[ns]
        if robot['map_known'] >= self.SCAN_THRESHOLD:
            x = random.uniform(-self.ARENA_HALF, self.ARENA_HALF)
            y = random.uniform(-self.ARENA_HALF, self.ARENA_HALF)
            yaw = random.uniform(-math.pi, math.pi)
            return x, y, yaw

        # Current pose in world coords (odom is anchored at spawn and aligned
        # with the spawn yaw — rotate odom coords into the world frame).
        pose = robot['pose']
        if pose is None:
            px, py, pyaw = self.spawns.get(ns, (0.0, 0.0, 0.0))
        else:
            xo, yo, yoaw = pose
            x0, y0, yaw0 = self.spawns.get(ns, (0.0, 0.0, 0.0))
            c, s = math.cos(yaw0), math.sin(yaw0)
            px = x0 + xo * c - yo * s
            py = y0 + xo * s + yo * c
            pyaw = (yoaw + yaw0) % (2.0 * math.pi)

        if random.random() < self.FOV_FILL_PROB:
            d = random.uniform(*self.FILL_DIST)
            h = random.uniform(-math.pi, math.pi)
        else:
            d = random.uniform(*self.FOV_DIST)
            cone = math.radians(self.FOV_CONE_DEG)
            h = random.uniform(-cone, cone)

        # Clamp inside the safe arena so the goal is always reachable.
        x = max(-self.ARENA_SAFE, min(self.ARENA_SAFE,
                                      px + d * math.cos(pyaw + h)))
        y = max(-self.ARENA_SAFE, min(self.ARENA_SAFE,
                                      py + d * math.sin(pyaw + h)))
        yaw = random.uniform(-math.pi, math.pi)
        return x, y, yaw

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
