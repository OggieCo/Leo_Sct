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
import csv
import math
import os
import signal
import time

from swarm_basics.goal_validation import load_obstacles, sanitize_tour
from swarm_basics.robot_config import ROBOT_POSITIONS, world_to_map
from swarm_basics.run_utils import get_run_dir

# ---------------------------------------------------------------------------
# PER-ROBOT FIXED TOURS - world (x, y) waypoints, one fixed sequence per rover.
# Ordered centre-out so the rover drives through already-scanned territory first.
# The tours share the inner ring (so rovers keep meeting in the human zone) but
# use complementary outer radii with the same goal count per robot, which keeps
# the A/B fair.
# Waypoints inside a static obstacle of the active scenario are snapped out at
# startup (see goal_validation.py; no-op for scenarios without obstacles).
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
    # --- 3- and 4-rover seeds (3_rovers_3_humans / 4_rovers_3_humans_obstacles):
    # two more complementary radius pairs (2.5/4.5 and 3.5/5.5).  With four
    # rovers the complementary spacing gets tight, so robot_2/robot_3 share a
    # few corner/diagonal waypoints with robot_0/robot_1 — edit freely.
    'robot_2': [
        # inner ring (rotated 4 steps vs robot_0; SHARED human zone)
        (0.0, 2.0),
        (-2.0, 2.0),
        (-2.0, 0.0),
        (-2.0, -2.0),
        (0.0, -2.0),
        (2.0, -2.0),
        (0.0, 0.0),
        (2.0, 0.0),
        # middle ring radius 2.5
        (2.5, 0.0),
        (2.5, 2.5),
        (0.0, 2.5),
        (-2.5, 2.5),
        (-2.5, 0.0),
        (-2.5, -2.5),
        (0.0, -2.5),
        (2.5, -2.5),
        # outer ring radius 4.5
        (4.5, 0.0),
        (4.5, 4.5),
        (0.0, 4.5),
        (-4.5, 4.5),
        (-4.5, 0.0),
        (-4.5, -4.5),
        (0.0, -4.5),
        (4.5, -4.5),
        # corners at 6.0
        (6.0, 6.0),
        (-6.0, 6.0),
        (-6.0, -6.0),
        (6.0, -6.0),
    ],
    'robot_3': [
        # inner ring (rotated 6 steps vs robot_0; SHARED human zone)
        (2.0, -2.0),
        (0.0, 0.0),
        (2.0, 0.0),
        (0.0, 2.0),
        (-2.0, 2.0),
        (-2.0, 0.0),
        (-2.0, -2.0),
        (0.0, -2.0),
        # middle ring radius 3.5
        (3.5, 0.0),
        (3.5, 3.5),
        (0.0, 3.5),
        (-3.5, 3.5),
        (-3.5, 0.0),
        (-3.5, -3.5),
        (0.0, -3.5),
        (3.5, -3.5),
        # outer ring radius 5.5
        (5.5, 0.0),
        (5.5, 5.5),
        (0.0, 5.5),
        (-5.5, 5.5),
        (-5.5, 0.0),
        (-5.5, -5.5),
        (0.0, -5.5),
        (5.5, -5.5),
        # corners at 6.5
        (6.5, 6.5),
        (-6.5, 6.5),
        (-6.5, -6.5),
        (6.5, -6.5),
    ],
    # 5-rover tour: robot_4 re-uses robot_2's radii {2.5, 4.5} rotated, plus the
    # 6.0 corners, so spacing and coverage stay comparable.
    'robot_4': [
        # inner ring (SAME 8 central cells, rotated 3 steps; SHARED human zone)
        (-2.0, 2.0),
        (-2.0, 0.0),
        (-2.0, -2.0),
        (0.0, -2.0),
        (2.0, -2.0),
        (0.0, 0.0),
        (2.0, 0.0),
        (0.0, 2.0),
        # middle ring radius 2.5 (rotated 2 steps vs robot_2's copy)
        (0.0, 2.5),
        (2.5, 2.5),
        (2.5, 0.0),
        (2.5, -2.5),
        (0.0, -2.5),
        (-2.5, -2.5),
        (-2.5, 0.0),
        (-2.5, 2.5),
        # outer ring radius 4.5 (rotated 4 steps vs robot_2's copy)
        (4.5, 4.5),
        (0.0, 4.5),
        (-4.5, 4.5),
        (-4.5, 0.0),
        (-4.5, -4.5),
        (0.0, -4.5),
        (4.5, -4.5),
        (4.5, 0.0),
        # corners at 6.0 (shared with robot_2)
        (6.0, 6.0),
        (-6.0, 6.0),
        (-6.0, -6.0),
        (6.0, -6.0),
    ],
}

# Fallback for any rover without a dedicated entry (keeps old behaviour).
FIXED_GOALS = FIXED_GOALS_BY_ROBOT['robot_0']


class FixedGoalPublisher(Node):
    # --- out-of-map guard -------------------------------------------------
    # Measure how much of the ARENA is known, not how much of the SLAM map is:
    # the canvas grows while exploring, so only cells inside the arena rectangle
    # count.  The threshold then means "we have mapped X % of the arena".
    ARENA_HALF = 7.3                # arena walls at +/-7.25 m in these scenarios
    SCAN_THRESHOLD = float(os.environ.get('SWARM_SCAN_THRESHOLD', '0.30'))
    # A fixed goal nearer than this bypasses the guard.
    SCAN_SAFE_DIST = float(os.environ.get('SWARM_SCAN_SAFE_DIST', '4.0'))
    SCAN_STEP_DIST = 3.0   # deterministic forward scan-goal distance (m)
    ARENA_SAFE = 6.0       # scan goals clamped inside the arena
    MAX_FAILS_BEFORE_SCAN = 3  # plan failures on one fixed goal -> scan first
    # After the scan rescue a goal gets one more failure battery, then the tour
    # advances (log SKIPPED_UNREACHABLE).
    MAX_FAILS_BEFORE_SKIP = 2 * MAX_FAILS_BEFORE_SCAN   # 6 attempts total
    # If Nav2 does not ACCEPT a goal within this long, the DDS response was lost -
    # retry instead of waiting forever.
    ACCEPT_TIMEOUT_S = float(os.environ.get('SWARM_ACCEPT_TIMEOUT_S', '40.0'))

    # How often a lost accept response may be re-sent before the attempt counts as a
    # failure (override: SWARM_ACCEPT_RETRIES).
    MAX_ACCEPT_RETRIES = int(os.environ.get('SWARM_ACCEPT_RETRIES', '1'))

    def __init__(self):
        super().__init__('fixed_goal_publisher')

        # Per-robot state: action client, stop publisher, goal handle, busy
        # flag, index into FIXED_GOALS, current odom pose and map known-frac.
        self.spawns = {ns: (x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS}

        # Scenario obstacle validation of the fixed tours (harness-side INPUT
        # validation, not navigation policy - every arm runs this identical code).
        self._tours = {}
        _obstacles = load_obstacles()
        _snapped = 0
        for _ns, _, _, _ in ROBOT_POSITIONS:
            _tour, _changes = sanitize_tour(
                FIXED_GOALS_BY_ROBOT.get(_ns, FIXED_GOALS), _obstacles)
            self._tours[_ns] = _tour
            _snapped += len(_changes)
            for _i, _old, _new, _oc in _changes:
                self.get_logger().warn(
                    f'[{_ns}] tour goal #{_i} ({_old[0]:.2f}, {_old[1]:.2f}) '
                    f'inside obstacle keepout @ ({_oc[0]:.2f}, {_oc[1]:.2f}) '
                    f'— snapped to ({_new[0]:.2f}, {_new[1]:.2f})')
        if _obstacles:
            self.get_logger().info(
                f'tour validation: {len(_obstacles)} static obstacle(s), '
                f'{_snapped} waypoint(s) snapped clear of keepout rings')
        else:
            self.get_logger().info(
                'tour validation: no static obstacles in this scenario — '
                'tours unchanged')

        # Resolve the shared run folder EAGERLY: start_experiment.sh starts this node
        # only after the Nav2 readiness gate, so a lazy resolve would find the run
        # folder already "old" and put goal_stats.csv into its own second folder.
        try:
            self._run_dir = get_run_dir()
        except OSError:
            self._run_dir = None   # fallback: retry on the first write

        # Arena window (world frame) used by the out-of-map guard: only cells
        # inside it count towards "how much of the arena is scanned".
        self._arena_min = -self.ARENA_HALF
        self._arena_max = self.ARENA_HALF

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
                'scan_used': False,    # forward-scan rescue spent on this goal?
                'pose': None,          # (x, y, yaw) in {ns}/odom frame
                'map_known': 0.0,      # known fraction of /{ns}/map
                'done_logged': False,
                # Goal-outcome counters -> goal_stats.csv (merged into
                # summary.csv by coverage_plotter, works for all three arms).
                'g_sent': 0, 'g_ok': 0, 'g_fail': 0, 'g_scan': 0,
                'attempt': 0,      # bumped per send; invalidates stale callbacks
                'accept_retries': 0,   # re-sends spent on THIS goal (bounded)
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
        for i, ns in enumerate(self.robots):
            self._schedule(2.0 + 0.75 * i, lambda ns=ns: self.send_goal(ns))

    def _schedule(self, delay, cb):
        """Run `cb()` once after `delay` seconds (self-cancelling timer)."""
        timer = self.create_timer(delay, lambda: self._fire_once(timer, cb))

    def _fire_once(self, timer, cb):
        timer.cancel()
        cb()

    def _goals(self, ns):
        """This robot's fixed tour (world waypoints), already validated
        against the active scenario's static obstacles (see __init__).
        Different route per rover, same goal count -> fair A/B."""
        return self._tours.get(ns, FIXED_GOALS)

    # ------------------------------------------------------------------ #
    # Perception helpers
    # ------------------------------------------------------------------ #
    def _map_cb(self, msg, ns):
        """Track the known (scanned) fraction of the ARENA.

        Only cells inside the arena window are counted, so the ratio is not
        diluted by the unknown margin that slam_toolbox keeps adding as the
        canvas grows (see the ARENA_HALF comment on the class).
        """
        robot = self.robots[ns]
        if not msg.data:
            return
        info = msg.info
        res = float(info.resolution)
        if res <= 0.0:
            return
        ox = info.origin.position.x
        oy = info.origin.position.y
        ix0 = max(0, int((self._arena_min - ox) / res))
        ix1 = min(info.width, int((self._arena_max - ox) / res) + 1)
        iy0 = max(0, int((self._arena_min - oy) / res))
        iy1 = min(info.height, int((self._arena_max - oy) / res) + 1)
        known = 0
        total = 0
        data = msg.data
        width = info.width
        for iy in range(iy0, iy1):
            row = iy * width
            for ix in range(ix0, ix1):
                total += 1
                if data[row + ix] >= 0:
                    known += 1
        if total:
            robot['map_known'] = known / total

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
        """Deterministic scan goal one step ALONG THE TOUR, towards the next waypoint.

        Steering at the next waypoint always makes progress; a fixed step along
        the current heading ping-pongs, because after a scan goal the rover often
        faces back the way it came.  Deterministic, so the A/B stays fair.
        """
        px, py, _ = self._world_pose(ns)
        fx, fy = self._goals(ns)[self.robots[ns]['idx']]
        dx, dy = fx - px, fy - py
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return fx, fy, False
        step = min(self.SCAN_STEP_DIST, dist)
        sx = max(-self.ARENA_SAFE, min(self.ARENA_SAFE, px + step * dx / dist))
        sy = max(-self.ARENA_SAFE, min(self.ARENA_SAFE, py + step * dy / dist))
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
        # deterministic forward scan goal before retrying it.
        if robot['force_scan']:
            robot['force_scan'] = False
            return self._forward_scan_goal(
                ns, f'goal #{idx} failed to plan '
                    f'{self.MAX_FAILS_BEFORE_SCAN} times — scanning forward '
                    f'before retrying')

        # Map scanned enough -> the whole arena is fair game.
        if robot['map_known'] >= self.SCAN_THRESHOLD:
            if not robot.get('unlock_logged'):
                robot['unlock_logged'] = True
                self.get_logger().info(
                    f'[{ns}] arena {robot["map_known"]*100:.0f}% known '
                    f'(>= {self.SCAN_THRESHOLD*100:.0f}%) — tour goals unlocked')
            return fx, fy, True

        # Goal close to the current pose -> bypass the guard, but ONLY when a safe
        # radius is configured.
        if (self.SCAN_SAFE_DIST > 0.0
                and math.hypot(fx - px, fy - py) <= self.SCAN_SAFE_DIST):
            return fx, fy, True

        # Out-of-map guard: forward scan goal, then the fixed goal is retried.
        return self._forward_scan_goal(
            ns, f'arena {robot["map_known"]*100:.0f}% known (need '
                f'{self.SCAN_THRESHOLD*100:.0f}%) — goal '
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
        # Goal orientation = the rover's CURRENT heading, so DWB's RotateToGoal critic
        # has nothing to rotate toward on arrival - the rover would otherwise spin in
        # place at every goal to match a fixed yaw.
        _, _, pyaw = self._world_pose(ns)
        mx, my, myaw = world_to_map(ns, x, y, pyaw)

        # Per-robot map frame (each rover has its own {ns}/map tree).
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
        if on_fixed:
            robot['g_sent'] += 1
        else:
            robot['g_scan'] += 1
        self._write_goal_stats()
        label = 'goal' if on_fixed else 'scan'
        self.get_logger().info(
            f'[{ns}] {label} #{idx}: world ({x:.1f}, {y:.1f}) -> '
            f'map ({mx:.1f}, {my:.1f})')
        robot['attempt'] += 1
        attempt = robot['attempt']
        future = ac.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, ns=ns, att=attempt: self._goal_response_cb(f, ns, att))
        # Watchdog: DDS can drop the accept response entirely (see
        # ACCEPT_TIMEOUT_S) — without this the node waits forever.
        self._schedule(self.ACCEPT_TIMEOUT_S,
                       lambda ns=ns, att=attempt, f=future:
                       self._accept_watchdog(ns, att, f))

    def _accept_watchdog(self, ns, attempt, future):
        """No accept response in time -> declare the attempt lost.

        The re-send is bounded by MAX_ACCEPT_RETRIES: one lost response may be
        re-sent, but it can no longer occupy the goal slot indefinitely.  Once
        the budget is spent the attempt counts as a failure and the normal
        failure battery (scan rescue, then skip) acts on that goal.
        """
        robot = self.robots[ns]
        if not robot['busy'] or attempt != robot['attempt'] or future.done():
            return
        robot['attempt'] += 1          # invalidate: a late response is ignored
        robot['busy'] = False
        robot['g_fail'] += 1
        self._write_goal_stats()
        if robot['accept_retries'] < self.MAX_ACCEPT_RETRIES:
            robot['accept_retries'] += 1
            self.get_logger().warn(
                f'[{ns}] no goal ACCEPT response after '
                f'{self.ACCEPT_TIMEOUT_S:.0f}s (DDS response lost?) — re-send '
                f'{robot["accept_retries"]}/{self.MAX_ACCEPT_RETRIES} in 5s')
            self._schedule(5.0, lambda: self.send_goal(ns))
            return
        self.get_logger().warn(
            f'[{ns}] no goal ACCEPT response after '
            f'{self.ACCEPT_TIMEOUT_S:.0f}s and the re-send budget is spent — '
            f'counting goal #{robot["idx"]} as failed')
        # g_fail was already counted above, so do not count it twice.
        self._handle_goal_failure(ns, count_fail=False)

    def _goal_response_cb(self, future, ns, attempt):
        robot = self.robots[ns]
        if attempt != robot['attempt']:
            # Response of a superseded (timed-out) attempt: if the server
            # actually accepted that goal, cancel it so the rover doesn't
            # drive an untracked goal.
            try:
                gh = future.result()
                if gh is not None and gh.accepted:
                    gh.cancel_goal_async()
                    self.get_logger().warn(
                        f'[{ns}] late-accepted goal from a lost attempt — canceled')
            except Exception:
                pass
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'[{ns}] Goal rejected by Nav2 — retrying in 5s')
            robot['g_fail'] += 1
            self._write_goal_stats()
            robot['busy'] = False
            self._schedule(5.0, lambda: self.send_goal(ns))
            return
        robot['accept_retries'] = 0    # accepted — fresh budget for this goal
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
                robot['scan_used'] = False
                robot['accept_retries'] = 0   # fresh budget for the next goal
                robot['g_ok'] += 1
                robot['idx'] += 1
                self.get_logger().info(
                    f'[{ns}] goal #{robot["idx"]-1} reached — next '
                    f'#{robot["idx"]}')
            else:
                self.get_logger().info(
                    f'[{ns}] scan goal reached — retrying goal '
                    f'#{robot["idx"]}')
            self._write_goal_stats()
            self._schedule(0.5, lambda: self.send_goal(ns))
        else:
            self.get_logger().warn(
                f'[{ns}] Goal finished with status {status} (not SUCCEEDED)')
            self._handle_goal_failure(ns)

    def _handle_goal_failure(self, ns, count_fail=True):
        """Failure battery for the goal currently being attempted.

        After MAX_FAILS_BEFORE_SCAN consecutive failures on a FIXED goal, one
        forward scan goal is sent to widen the map; after MAX_FAILS_BEFORE_SKIP
        the goal is skipped and the tour advances.  Shared by the result
        callback and by the accept watchdog (a lost accept response counts as a
        failure once its re-send budget is spent), so an unreachable or
        unacknowledged goal can never eat the whole run.
        """
        robot = self.robots[ns]
        robot['fail_count'] += 1
        if count_fail and robot['on_fixed_goal']:
            robot['g_fail'] += 1
        self._write_goal_stats()
        if (robot['on_fixed_goal'] and
                robot['fail_count'] >= self.MAX_FAILS_BEFORE_SCAN):
            if not robot['scan_used']:
                # Stop retrying an unreachable fixed goal in place: send ONE
                # forward scan goal to widen the map, then retry.
                robot['scan_used'] = True
                robot['force_scan'] = True
                robot['fail_count'] = 0
                self.get_logger().warn(
                    f'[{ns}] goal #{robot["idx"]} failed to plan '
                    f'{self.MAX_FAILS_BEFORE_SCAN} times — will scan '
                    f'forward before retrying')
            else:
                # Even the scan-rescued retry survived a full failure battery ->
                # genuinely unreachable for this arm: record it and advance the tour
                # (safety net; with sanitized tours this should never fire).
                robot['idx'] += 1
                robot['fail_count'] = 0
                robot['scan_used'] = False
                robot['accept_retries'] = 0
                self.get_logger().warn(
                    f'[{ns}] goal #{robot["idx"] - 1} '
                    f'SKIPPED_UNREACHABLE after '
                    f'{self.MAX_FAILS_BEFORE_SKIP} failed attempts — '
                    f'advancing to #{robot["idx"]}')
                self._write_goal_stats()
                self._schedule(0.5, lambda: self.send_goal(ns))
                return
        self._schedule(5.0, lambda: self.send_goal(ns))

    def _write_goal_stats(self):
        """Per-robot goal outcomes -> <run_dir>/goal_stats.csv.

        Works for every arm (Nav2 / BT / BT+LLM — the goal driver is shared):
        goals_sent / goals_succeeded / goal_failures refer to the REAL tour goals
        (goal_failures counts failed ATTEMPTS, so one goal can add several),
        scan_goals counts
        the temporary forward-scan goals.  Rewritten on every change (tiny
        file) so a Ctrl-C can never lose the counts; coverage_plotter merges
        these rows into summary.csv.
        """
        try:
            if self._run_dir is None:
                # Fallback if the eager resolve in __init__ failed: resolve ONCE and
                # reuse for every later write - a fresh get_run_dir() per write
                # splintered one run across folders.
                self._run_dir = get_run_dir()
            path = self._run_dir / 'goal_stats.csv'
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['robot_id', 'goals_sent', 'goals_succeeded',
                            'goal_failures', 'scan_goals'])
                for ns in self.robots:
                    r = self.robots[ns]
                    w.writerow([ns, r.get('g_sent', 0), r.get('g_ok', 0),
                                r.get('g_fail', 0), r.get('g_scan', 0)])
        except OSError as e:
            self.get_logger().warn(f'could not write goal_stats.csv: {e}')

    def killswitch(self):
        """Stop every rover on shutdown: cancel active Nav2 goals and flush
        zero-velocity commands to Gazebo BEFORE rclpy tears down the context
        (publishing in destroy_node is too late)."""
        self.get_logger().info('Stopping robot(s)...')
        self._write_goal_stats()   # final counts before the run folder closes
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
