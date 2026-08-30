#!/usr/bin/env python3
"""Publishes a FIXED, predefined tour of navigation goals for 1..N robots.

Identical goals for every condition -> a fair, repeatable A/B test: run it
under BT (reactive) and BT+LLM and compare like-for-like instead of relying on
random goals.  The run folders are routed automatically by SWARM_RUN_MODE (set
from enable_llm in spawn_multi_robots.launch.py):
    enable_llm:=true  -> results/coverage_logs/BT_LLM/
    enable_llm:=false -> results/coverage_logs/BT/

Each robot walks its OWN fixed tour in FIXED_GOALS_BY_ROBOT (different route
per rover, SAME goal count), one goal at a time: a new goal is only sent once
the previous one was reached, so every tour is deterministic and both A/B
conditions (BT vs BT+LLM) see identical per-robot goal streams.

Lidar "out-of-map" guard (the lesson from random_goals.py): a far goal is only
sent once the rover's own SLAM map is scanned enough (known-fraction >=
SCAN_THRESHOLD) OR the goal is close to the current pose.  Otherwise the rover
gets a short DETERMINISTIC forward scan goal (3 m straight ahead) to widen the
map before retrying the fixed goal — no blind out-of-sight goals into
unscanned space, and no randomness injected into the A/B.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import signal
import time

from swarm_basics.robot_config import ROBOT_POSITIONS, world_to_map

# ---------------------------------------------------------------------------
# PER-ROBOT FIXED TOURS — world (x, y) waypoints, one fixed sequence per rover.
# Ordered centre-out (square spiral) so the rover always drives through
# already-scanned territory: central goals first (where the humans roam),
# then the outer rings, then the four corners.
#
# The two tours SHARE the inner ring (same 8 central cells, different
# starting point) so the rovers keep crossing paths in the human zone and
# interact socially; the middle/outer rings stay COMPLEMENTARY (robot_0 on
# EVEN radii {4, 6, +6.5 corners}, robot_1 on ODD radii {3, 5, +5.5 corners})
# so the tours remain distinct overall.  Same goal COUNT (28) and same arena
# coverage per robot -> the A/B stays fair.  Edit freely — every run uses
# these exact lists.
# ---------------------------------------------------------------------------
FIXED_GOALS_BY_ROBOT = {
    'robot_0': [
        # --- inner ring (central human zone; SHARED with robot_1) ---
        (0.0, 0.0),
        (2.0, 0.0),
        (0.0, 2.0),
        (-2.0, 2.0),
        (-2.0, 0.0),
        (-2.0, -2.0),
        (0.0, -2.0),
        (2.0, -2.0),
        # --- middle ring ---
        (4.0, 0.0),
        (4.0, 4.0),
        (0.0, 4.0),
        (-4.0, 4.0),
        (-4.0, 0.0),
        (-4.0, -4.0),
        (0.0, -4.0),
        (4.0, -4.0),
        # --- outer ring (near the edges) ---
        (6.0, 0.0),
        (6.0, 6.0),
        (0.0, 6.0),
        (-6.0, 6.0),
        (-6.0, 0.0),
        (-6.0, -6.0),
        (0.0, -6.0),
        (6.0, -6.0),
        # --- the four corners (walls at +/-7.25, safe goal at +/-6.5) ---
        (6.5, 6.5),
        (-6.5, 6.5),
        (-6.5, -6.5),
        (6.5, -6.5),
    ],
    'robot_1': [
        # --- inner ring (SAME 8 cells as robot_0, rotated 2 steps so they do
        #      not both beeline the exact same cell at t=0; paths cross in
        #      the central human zone) ---
        (-2.0, 0.0),
        (-2.0, -2.0),
        (0.0, -2.0),
        (2.0, -2.0),
        (0.0, 0.0),
        (2.0, 0.0),
        (0.0, 2.0),
        (-2.0, 2.0),
        # --- middle ring (radius 3, complementary to robot_0's radius 4) ---
        (3.0, 0.0),
        (3.0, 3.0),
        (0.0, 3.0),
        (-3.0, 3.0),
        (-3.0, 0.0),
        (-3.0, -3.0),
        (0.0, -3.0),
        (3.0, -3.0),
        # --- outer ring (radius 5, complementary to robot_0's radius 6) ---
        (5.0, 0.0),
        (5.0, 5.0),
        (0.0, 5.0),
        (-5.0, 5.0),
        (-5.0, 0.0),
        (-5.0, -5.0),
        (0.0, -5.0),
        (5.0, -5.0),
        # --- the four corners (safe goal at +/-5.5, complementary to 6.5) ---
        (5.5, 5.5),
        (-5.5, 5.5),
        (-5.5, -5.5),
        (5.5, -5.5),
    ],
}

# Fallback for any rover without a dedicated entry (keeps old behaviour).
FIXED_GOALS = FIXED_GOALS_BY_ROBOT['robot_0']


class FixedGoalPublisher(Node):
    SCAN_THRESHOLD = 0.6   # known fraction of /{ns}/map that unlocks far goals
    SCAN_SAFE_DIST = 5.0   # a fixed goal closer than this is always allowed
    SCAN_STEP_DIST = 3.0   # deterministic forward scan-goal distance (m)
    ARENA_SAFE = 6.0       # scan goals clamped inside the arena
    MAX_FAILS_BEFORE_SCAN = 3  # plan failures on one fixed goal -> scan first

    def __init__(self):
        super().__init__('fixed_goal_publisher')

        # Per-robot state: action client, stop publisher, goal handle, busy
        # flag, index into FIXED_GOALS, current odom pose and map known-frac.
        self.spawns = {ns: (x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS}
        self.robots = {}
        for ns, _, _, _ in ROBOT_POSITIONS:
            self.robots[ns] = {
                'action_client': ActionClient(
                    self, NavigateToPose, f'/{ns}/navigate_to_pose'),
                'cmd_pub': self.create_publisher(Twist, f'/{ns}/cmd_vel', 10),
                'goal_handle': None,
                'busy': False,
                'idx': 0,              # next index into FIXED_GOALS
                'on_fixed_goal': False,  # is the goal in flight a FIXED goal?
                'fail_count': 0,       # consecutive non-SUCCEEDED results
                'force_scan': False,   # next send must be a forward scan goal
                'pose': None,          # (x, y, yaw) in {ns}/odom frame
                'map_known': 0.0,      # known fraction of /{ns}/map
                'done_logged': False,
            }
            # SLAM occupancy grid -> "is the arena scanned enough for far goals?"
            self.create_subscription(
                OccupancyGrid, f'/{ns}/map',
                lambda msg, ns=ns: self._map_cb(msg, ns), 10)
            # Odom -> current pose (for the scan guard + forward scan goals)
            self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda msg, ns=ns: self._odom_cb(msg, ns), 10)

        self.get_logger().info(
            f'FixedGoalPublisher started for {len(self.robots)} robot(s) — '
            f'per-robot fixed tours: ' +
            ', '.join(f'{ns}={len(self._goals(ns))} goals'
                     for ns in self.robots))

        # Kick off the first goal for each robot once the node has spun up.
        for ns in self.robots:
            self._schedule(2.0, lambda ns=ns: self.send_goal(ns))

    def _schedule(self, delay, cb):
        """Run `cb()` once after `delay` seconds (self-cancelling timer)."""
        timer = self.create_timer(delay, lambda: self._fire_once(timer, cb))

    def _fire_once(self, timer, cb):
        timer.cancel()
        cb()

    def _goals(self, ns):
        """This robot's fixed tour (world waypoints).  Different route per
        rover, same goal count -> fair A/B with disjoint waypoints."""
        return FIXED_GOALS_BY_ROBOT.get(ns, FIXED_GOALS)

    # ------------------------------------------------------------------ #
    # Perception helpers
    # ------------------------------------------------------------------ #
    def _map_cb(self, msg, ns):
        """Track the known (scanned) fraction of this rover's SLAM map."""
        robot = self.robots[ns]
        if not msg.data:
            return
        known = sum(1 for v in msg.data if v >= 0)
        robot['map_known'] = known / len(msg.data)

    def _odom_cb(self, msg, ns):
        """Remember the rover's current pose in its odom frame."""
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robots[ns]['pose'] = (p.position.x, p.position.y, yaw)

    def _world_pose(self, ns):
        """Current pose of robot `ns` in WORLD coords.

        Odom is anchored at spawn and aligned with the spawn yaw, so rotate
        odom coords into the world frame (same trick as random_goals.py).
        """
        pose = self.robots[ns]['pose']
        if pose is None:
            return self.spawns.get(ns, (0.0, 0.0, 0.0))
        xo, yo, yoaw = pose
        x0, y0, yaw0 = self.spawns.get(ns, (0.0, 0.0, 0.0))
        c, s = math.cos(yaw0), math.sin(yaw0)
        px = x0 + xo * c - yo * s
        py = y0 + xo * s + yo * c
        pyaw = (yoaw + yaw0) % (2.0 * math.pi)
        return px, py, pyaw

    def _forward_scan_goal(self, ns, reason):
        """Deterministic forward scan goal (no randomness -> A/B stays fair):
        widen the map a little, then the fixed goal is retried.  Used by the
        out-of-map guard AND the repeated plan-failure fallback."""
        px, py, pyaw = self._world_pose(ns)
        sx = max(-self.ARENA_SAFE, min(self.ARENA_SAFE,
                                       px + self.SCAN_STEP_DIST * math.cos(pyaw)))
        sy = max(-self.ARENA_SAFE, min(self.ARENA_SAFE,
                                       py + self.SCAN_STEP_DIST * math.sin(pyaw)))
        self.get_logger().info(f'[{ns}] {reason}')
        return sx, sy, False

    def _pick_goal(self, ns):
        """Decide the next goal for robot `ns`.

        Returns (x, y, on_fixed).  `on_fixed=True` means this is a real tour
        goal and the index will advance on success; `False` means it is a
        temporary forward scan goal (index unchanged — the fixed goal is
        retried afterwards).  Scan goals come from the out-of-map guard OR
        from repeated plan failures (see _result_cb).
        """
        robot = self.robots[ns]
        idx = robot['idx']
        fx, fy = self._goals(ns)[idx]
        px, py, _ = self._world_pose(ns)

        # Repeated plan failures on this fixed goal -> widen the map with a
        # deterministic forward scan goal before retrying it (the planner
        # keeps aborting because the goal cell is still unknown/blocked at
        # the map edge — run_2026-08-27_17-39-03 / _17-49-23: a rover retried
        # the same unreachable goal every 5 s for ~80 s).
        if robot['force_scan']:
            robot['force_scan'] = False
            return self._forward_scan_goal(
                ns, f'goal #{idx} failed to plan '
                    f'{self.MAX_FAILS_BEFORE_SCAN} times — scanning forward '
                    f'before retrying')

        # Map scanned enough -> the whole arena is fair game.
        if robot['map_known'] >= self.SCAN_THRESHOLD:
            return fx, fy, True

        # Goal close to the current pose -> already inside the scanned
        # vicinity, safe to go straight there.
        if math.hypot(fx - px, fy - py) <= self.SCAN_SAFE_DIST:
            return fx, fy, True

        # Out-of-map guard: forward scan goal, then the fixed goal is retried.
        return self._forward_scan_goal(
            ns, f'map {robot["map_known"]*100:.0f}% known — goal '
                f'({fx:.1f}, {fy:.1f}) too far into unknown space; scanning '
                f'forward first')

    # ------------------------------------------------------------------ #
    # Goal flow (same action-client pattern as random_goals.py)
    # ------------------------------------------------------------------ #
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

        idx = robot['idx']
        if idx >= len(self._goals(ns)):
            if not robot['done_logged']:
                robot['done_logged'] = True
                self.get_logger().info(
                    f'[{ns}] Tour complete — all '
                    f'{len(self._goals(ns))} fixed goals visited. Idle.')
            return

        x, y, on_fixed = self._pick_goal(ns)
        # Goal orientation = the rover's CURRENT heading (world), so DWB's
        # RotateToGoal critic has nothing to rotate toward on arrival — the
        # rover would otherwise spin in place at every goal to match a fixed
        # yaw (adhesive wheels cannot zero-turn; seen in the LLM fixed-goals
        # run).  Applied to scan goals too (same send path).
        _, _, pyaw = self._world_pose(ns)
        mx, my, myaw = world_to_map(ns, x, y, pyaw)

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
        robot['on_fixed_goal'] = on_fixed
        label = 'goal' if on_fixed else 'scan'
        self.get_logger().info(
            f'[{ns}] {label} #{idx}: world ({x:.1f}, {y:.1f}) -> '
            f'map ({mx:.1f}, {my:.1f})')
        future = ac.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, ns=ns: self._goal_response_cb(f, ns))

    def _goal_response_cb(self, future, ns):
        robot = self.robots[ns]
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'[{ns}] Goal rejected by Nav2 — retrying in 5s')
            robot['busy'] = False
            self._schedule(5.0, lambda: self.send_goal(ns))
            return
        robot['goal_handle'] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, ns=ns: self._result_cb(f, ns))

    def _result_cb(self, future, ns):
        robot = self.robots[ns]
        result = future.result()
        robot['busy'] = False
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            robot['fail_count'] = 0
            if robot['on_fixed_goal']:
                robot['idx'] += 1
                self.get_logger().info(
                    f'[{ns}] goal #{robot["idx"]-1} reached — next '
                    f'#{robot["idx"]}')
            else:
                self.get_logger().info(
                    f'[{ns}] scan goal reached — retrying goal '
                    f'#{robot["idx"]}')
            self._schedule(0.5, lambda: self.send_goal(ns))
        else:
            robot['fail_count'] += 1
            if (robot['on_fixed_goal'] and
                    robot['fail_count'] >= self.MAX_FAILS_BEFORE_SCAN):
                # Stop retrying an unreachable fixed goal in place: send a
                # forward scan goal once to widen the map, then retry.
                robot['force_scan'] = True
                robot['fail_count'] = 0
                self.get_logger().warn(
                    f'[{ns}] goal #{robot["idx"]} failed to plan '
                    f'{self.MAX_FAILS_BEFORE_SCAN} times — will scan forward '
                    f'before retrying')
            else:
                self.get_logger().warn(
                    f'[{ns}] Goal finished with status {status} (not '
                    f'SUCCEEDED) — retrying in 5s')
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
    node = FixedGoalPublisher()

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
        node.get_logger().info('Shutting down fixed goal publisher.')
    finally:
        try:
            node.killswitch()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
