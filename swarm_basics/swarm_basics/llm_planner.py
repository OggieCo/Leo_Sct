#!/usr/bin/env python3
"""llm_planner — LLM-based high-level decision layer for social navigation.

Subscribes to the perception topics (robot proximity + human detection) and
calls OpenAI (gpt-4o-mini) ONLY when the situation materially changes and
something actually needs deciding (event-driven).  Publishes:

  llm_action      (std_msgs/String)  proceed | yield | slow | arc_left | arc_right | stop
  llm_reason      (std_msgs/String)  one-line natural-language justification
  llm_speed_scale (std_msgs/Float32) 0..1 soft speed cap for the 'slow' action

Cost guards:
  * event-driven (only on a changed situation signature),
  * cooldown between calls,
  * no API call at all when the path is clear,
  * per-call token usage + estimated cost logged (transparent for the thesis),
  * per-run CSV: llm_usage.csv (per call) + llm_summary.csv (totals) written
    into the same run folder as the coverage/social logs.

Uses plain stdlib urllib (the bundled openai client in this container is
broken), so no extra dependency is needed.  The key is read from OPENAI_API_KEY
or the repo .env (gitignored).
"""

import csv
import fcntl
import json
import math
import os
import signal
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from swarm_basics.run_utils import get_run_dir

MODEL = "gpt-4o-mini"
PRICE_IN = 0.15 / 1e6      # USD per input token (gpt-4o-mini)
PRICE_OUT = 0.60 / 1e6     # USD per output token

ACTIONS = ("proceed", "yield", "slow", "arc_left", "arc_right", "stop")

SYSTEM_PROMPT = """You are the social decision layer of a mobile robot navigating around other robots and humans. Choose exactly ONE action:
- "proceed": continue at full speed along the current path (default when there is no conflict).
- "slow": keep driving but at reduced speed — use when someone is approaching but does NOT block you yet (anticipatory caution). The target speed is set separately (llm_speed_scale).
- "yield": stop and let the other pass. Yield to a robot ONLY when the situation report says YOU are the YIELDING robot in the encounter; also yield to a human that is CLOSE (within 1.5 m) and ahead of you or crossing your path. NEVER yield when you HAVE the right-of-way.  NOTE: for a head-on (robot dead ahead) do NOT yield — use "arc_left"/"arc_right" to pass.
- "arc_left" / "arc_right": the head-on passing maneuver — drive a smooth curved detour to that side and continue toward the goal. Use it directly when another robot is dead ahead (head-on), and as a fallback after having yielded long enough and the other is STILL blocking. NEVER an in-place spin.
- "stop": emergency stop (e.g., collision imminent).
Rules: humans always have priority; prefer to wait for the other to pass and then continue straight (minimal deviation); never suggest in-place rotations.
On a head-on (robot dead ahead): NEVER stop and wait — pick "arc_left" or "arc_right" immediately so both of you curve past each other and continue toward the goal. NEVER pick "proceed" when head-on and DCA < 1.0 m (imminent collision).  "yield" is only for side/crossing conflicts or humans.
A robot BESIDE or BEHIND you (bearing magnitude > 60 deg) has ALREADY passed — NEVER yield to it; choose "proceed".
RIGHT-OF-WAY IS LOCKED FOR THE WHOLE ENCOUNTER (right-hand traffic: the robot on the LEFT yields, the robot on the RIGHT has the way). The situation report tells you once, at first contact, whether YOU are the YIELDING robot or HAVE the right-of-way — follow it and do NOT change your mind mid-pass. If you HAVE the way: NEVER yield, even if the other robot is crossing, faster, or seems to be on your right as it passes. If you are YIELDING: stop and let it pass, then resume once it is clearly past (bearing > 60 deg).
STALL ESCAPE: ONLY the YIELDING robot may break a stuck pass. If you are the YIELDING robot and the other robot is nearly stationary (speed < 0.1 m/s) directly ahead for 3+ seconds, the pass is STUCK — stop yielding and choose "arc_left"/"arc_right" to drive around it (prefer the side with more free space). If you HAVE the right-of-way, NEVER escape on your own — keep your path; the yielding robot will move aside (the recovery layer handles it if it never does).
Use the motion/free-space hints: if a human is approaching (range rate negative) but still >1.5 m away, "slow" is often better than a full stop. A human reported "not close (>1.5 m)" is NOT a hard conflict — prefer "proceed" or "slow", not "yield".
Respond with a single JSON object, no markdown, no extra text: {"action": "proceed|yield|slow|arc_left|arc_right|stop", "reason": "short reason"}"""


def load_api_key():
    """OPENAI_API_KEY env first, then the repo .env (gitignored)."""
    key = os.environ.get("OPENAI_API_KEY", "")
    if key:
        return key.strip()
    for path in ("/root/ros2_ws/src/.env", ".env", "/root/.env"):
        try:
            with open(path) as f:
                for line in f:
                    if line.startswith("OPENAI_API_KEY="):
                        return line.split("=", 1)[1].strip()
        except OSError:
            continue
    return ""


class LlmPlanner(Node):
    def __init__(self):
        super().__init__("llm_planner")
        self.ns = self.get_namespace().strip("/") or "root"

        self.declare_parameter("model", MODEL)
        self.declare_parameter("cooldown_s", 2.5)   # min gap between calls
        self.declare_parameter("max_tokens", 80)
        self.declare_parameter("tick_hz", 4.0)

        self._model = self.get_parameter("model").value
        self._cooldown = self.get_parameter("cooldown_s").value
        self._max_tokens = self.get_parameter("max_tokens").value
        self._key = load_api_key()

        # --- perception state ---
        self._robot_close = False
        self._robot_angle = 0.0
        self._robot_dca = 100.0
        self._robot_faster = False
        self._robot_dist = float("inf")
        self._robot_id = ""
        self._human_det = False
        self._human_close = False
        self._human_dist = -1.0
        self._human_angle = 0.0
        self._own_speed = 0.0
        self._other_speed = 0.0
        self._human_hist = []       # [(t, dist)] for range-rate
        self._human_ang_hist = []   # [(t, angle)] for crossing estimate
        self._free_front = float("inf")
        self._free_left = float("inf")
        self._free_right = float("inf")

        # --- decision state ---
        self._last_signature = None
        self._last_call_t = 0.0
        self._last_action = "proceed"
        self._last_reason = "no conflict yet"
        self._busy = False
        # Right-of-way lock: committed ONCE at first robot contact (like real
        # traffic) so the LLM does not re-evaluate left/right mid-pass and
        # freeze the right-of-way robot (run_2026-08-24_13-12-27).
        self._row = None             # None | "headon" | "yield" | "way"
        self._row_clear_since = 0.0
        # Deterministic release latch: once the pass has cleared, a stale
        # "yield" from an in-flight _decide thread must not re-stop the robot
        # (run_2026-08-24_14-39-39: stop-go at 41.8 s).  Reset per encounter.
        self._det_released = False
        # Two-stage yield: while the other robot is still farther than this
        # (m) away and we are still moving, publish "slow" (graceful
        # deceleration + a closer approach) instead of an abrupt full stop at
        # the 2.4 m proximity trigger (run_2026-08-24_14-42-57: robot_0
        # slammed to a stop at 2.2 m separation).  The full stop commits only
        # at the yield point.
        self._yield_stop_dist = 1.75
        self._yield_started_t = 0.0  # when the current yield action began
        self._stall_since = 0.0      # how long the other robot has stalled ahead
        self._rob_ang_prev = None    # previous other-robot bearing (stall check)
        self._rob_ang_prev_t = 0.0
        self._dist_prev = None       # previous other-robot range (range-rate)
        self._dist_prev_t = 0.0
        self._range_rate = 0.0       # + = other robot moving away

        # --- CSV usage logging (same run folder as coverage/social) ---
        self._start_time = time.time()
        self._csv_dir = str(get_run_dir())
        os.makedirs(self._csv_dir, exist_ok=True)
        self._usage_path = os.path.join(self._csv_dir, "llm_usage.csv")
        self._summary_path = os.path.join(self._csv_dir, "llm_summary.csv")
        self._calls = 0
        self._tokens_in = 0
        self._tokens_out = 0
        self._cost = 0.0
        self._summary_written = False
        # Append-mode + lock: BOTH robots share this usage file.  Opening with
        # "w" let the second llm_planner TRUNCATE the first's rows, and
        # concurrent flush() interleaved lines (corrupted llm_usage.csv).
        self._usage_file = open(self._usage_path, "a", newline="")
        self._usage_writer = csv.writer(self._usage_file)
        fcntl.flock(self._usage_file, fcntl.LOCK_EX)
        if os.path.getsize(self._usage_path) == 0:
            self._usage_writer.writerow([
                "timestamp", "elapsed_sec", "robot_id", "action", "reason",
                "in_tokens", "out_tokens", "total_tokens", "cost_usd"])
        self._usage_file.flush()
        fcntl.flock(self._usage_file, fcntl.LOCK_UN)

        # --- subscriptions ---
        self.create_subscription(Bool, "robot_close", self._cb_robot_close, 10)
        self.create_subscription(Float32, "robot_angle", self._cb_robot_angle, 10)
        self.create_subscription(Float32, "robot_dca", self._cb_robot_dca, 10)
        self.create_subscription(Bool, "robot_faster", self._cb_robot_faster, 10)
        self.create_subscription(Float32, "nearest_robot_dist", self._cb_robot_dist, 10)
        self.create_subscription(String, "nearest_robot_id", self._cb_robot_id, 10)
        self.create_subscription(Bool, "human_detected", self._cb_human_det, 10)
        self.create_subscription(Bool, "human_close", self._cb_human_close, 10)
        self.create_subscription(Float32, "human_distance", self._cb_human_dist, 10)
        self.create_subscription(Float32, "human_angle", self._cb_human_angle, 10)
        self.create_subscription(Float32, "other_speed", self._cb_other_speed, 10)
        self.create_subscription(LaserScan, "lidar/scan_clean", self._cb_scan, 10)
        self.create_subscription(Odometry, "odom", self._cb_odom, 10)

        # --- publishers ---
        self._pub_action = self.create_publisher(String, "llm_action", 10)
        self._pub_reason = self.create_publisher(String, "llm_reason", 10)
        self._pub_speed_scale = self.create_publisher(Float32, "llm_speed_scale", 10)

        # --- event-driven ticker ---
        self.create_timer(1.0 / self.get_parameter("tick_hz").value, self._tick)

        self.get_logger().info(
            f"LlmPlanner [{self.ns}]: model={self._model} cooldown={self._cooldown}s "
            f"key={'set' if self._key else 'MISSING (reactive fallback only)'}")
        self._publish()

    # --- callbacks -------------------------------------------------------
    def _cb_robot_close(self, m): self._robot_close = bool(m.data)
    def _cb_robot_angle(self, m): self._robot_angle = float(m.data)
    def _cb_robot_dca(self, m): self._robot_dca = float(m.data)
    def _cb_robot_faster(self, m): self._robot_faster = bool(m.data)
    def _cb_robot_dist(self, m): self._robot_dist = float(m.data)
    def _cb_robot_id(self, m): self._robot_id = m.data
    def _cb_human_det(self, m): self._human_det = bool(m.data)
    def _cb_human_close(self, m): self._human_close = bool(m.data)
    def _cb_human_dist(self, m):
        self._human_dist = float(m.data)
        now = time.time()
        self._human_hist.append((now, self._human_dist))
        while self._human_hist and now - self._human_hist[0][0] > 1.5:
            self._human_hist.pop(0)
    def _cb_human_angle(self, m):
        self._human_angle = float(m.data)
        now = time.time()
        self._human_ang_hist.append((now, self._human_angle))
        while self._human_ang_hist and now - self._human_ang_hist[0][0] > 1.5:
            self._human_ang_hist.pop(0)
    def _cb_other_speed(self, m): self._other_speed = float(m.data)
    def _cb_scan(self, m):
        """Sector free-space (min range in front / left / right) for the LLM."""
        front = left = right = float("inf")
        amin = m.angle_min
        inc = m.angle_increment
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            deg = math.degrees(amin + i * inc)
            if -30.0 <= deg <= 30.0:
                front = min(front, r)
            elif 30.0 < deg <= 90.0:
                left = min(left, r)
            elif -90.0 <= deg < -30.0:
                right = min(right, r)
        self._free_front, self._free_left, self._free_right = front, left, right
    def _cb_odom(self, m): self._own_speed = m.twist.twist.linear.x

    def _human_motion(self):
        """Return (range_rate m/s, label). range_rate < 0 = approaching."""
        h = self._human_hist
        rate = 0.0
        if len(h) >= 2 and h[-1][0] - h[0][0] > 0.2:
            rate = (h[-1][1] - h[0][1]) / (h[-1][0] - h[0][0])
        ang_rate = 0.0
        a = self._human_ang_hist
        if len(a) >= 2 and a[-1][0] - a[0][0] > 0.2:
            ang_rate = (a[-1][1] - a[0][1]) / (a[-1][0] - a[0][0])
        if rate < -0.1:
            label = "approaching me"
        elif abs(ang_rate) > 20.0:      # deg/s, bearing sweeping -> crossing
            label = "crossing my path"
        elif rate > 0.1:
            label = "moving away"
        else:
            label = "standing still"
        return rate, label

    # --- right-of-way lock ---------------------------------------------------
    def _update_row(self):
        """Commit the right-of-way ONCE at first robot contact (like real
        traffic), so the LLM does NOT re-evaluate left/right every tick and
        freeze the right-of-way robot mid-pass (run_2026-08-24_13-12-27: the
        right-of-way robot yielded twice and froze ~6 s)."""
        now = time.time()
        if not self._robot_close:
            if self._row is not None:
                if self._row_clear_since == 0.0:
                    self._row_clear_since = now
                elif now - self._row_clear_since >= 2.0:
                    self._row = None
                    self._row_clear_since = 0.0
                    self._yield_started_t = 0.0
                    self._stall_since = 0.0
                    self._det_released = False
            return
        self._row_clear_since = 0.0
        if self._row is None:
            # New encounter -> arm the deterministic release again.
            self._det_released = False
            if abs(self._robot_angle) <= 10.0:
                self._row = "headon"   # true head-on: both arc
            elif self._robot_angle < 0:
                self._row = "yield"    # other on our right -> we yield
            else:
                self._row = "way"      # other on our left -> we have the way

    def _update_stall(self):
        """Track how long the other robot has been GENUINELY stalled directly
        ahead: low speed AND a static bearing (it is not sweeping past).
        A robot that is merely passing slowly (e.g. arcing) has a moving
        bearing and is NOT stalled — otherwise the yielding robot wrongly
        escapes while the other is mid-arc and they tangle
        (run_2026-08-24_13-32-46)."""
        now = time.time()
        # Range-rate of the other robot (+ = moving away) — used to release
        # the yield once it has passed our front and is receding.
        if (self._dist_prev is not None and now - self._dist_prev_t > 0.2):
            self._range_rate = (self._robot_dist - self._dist_prev) / \
                (now - self._dist_prev_t)
        else:
            self._range_rate = 0.0
        self._dist_prev = self._robot_dist
        self._dist_prev_t = now
        ang_rate = 0.0
        if (self._rob_ang_prev is not None and
                now - self._rob_ang_prev_t > 0.2):
            ang_rate = abs(self._robot_angle - self._rob_ang_prev) / \
                (now - self._rob_ang_prev_t)
        self._rob_ang_prev = self._robot_angle
        self._rob_ang_prev_t = now
        if (self._robot_close and abs(self._robot_angle) <= 60.0
                and self._other_speed < 0.1 and ang_rate < 8.0):
            if self._stall_since == 0.0:
                self._stall_since = now
        else:
            self._stall_since = 0.0

    def _yield_cleared(self):
        """DETERMINISTIC yield release for the YIELDING robot (bypasses the
        LLM): the pass is over once the other robot has left our front
        corridor — bearing past ~50 deg, OR it is receding (range-rate > 0)
        past a lateral margin of ~0.8 m.  Releasing here (not via the LLM)
        removes the long tail where the yielding robot hangs on while the
        other drives away (run_2026-08-24_14-15-51: ~11 s yield).  Easily
        reverted: remove the two call sites in _tick and _decide."""
        a = abs(self._robot_angle)
        if a > 50.0:
            return True
        if a <= 35.0:
            return False
        lat = self._robot_dist * math.sin(math.radians(self._robot_angle))
        return self._range_rate > 0.05 and abs(lat) > 0.8

    # --- event-driven tick -------------------------------------------------
    def _signature(self):
        rr, _ = self._human_motion()
        return (
            round(self._robot_dist, 1), round(self._robot_angle, 5),
            round(self._robot_dca, 1), self._robot_faster, self._robot_close,
            self._robot_id, round(self._other_speed, 2),
            self._human_det, self._human_close, round(self._human_dist, 1),
            round(self._human_angle, 5), round(rr, 2),
            round(self._free_front, 1),
        )

    def _tick(self):
        self._update_row()
        self._update_stall()
        # Deterministic release: don't wait for the LLM/cooldown once the
        # pass has clearly cleared.
        if (self._row == "yield" and self._yield_cleared() and
                self._last_action != "proceed"):
            self._last_action = "proceed"
            self._last_reason = "pass cleared — resuming (deterministic release)"
            self._det_released = True
            self._publish()
        # TWO-STAGE YIELD (deterministic, every tick): while the other robot
        # is still farther than _yield_stop_dist and we are still moving,
        # publish "slow" (decelerate + creep closer) instead of a full stop.
        # The full "yield" stop commits only at the yield point, so the robot
        # no longer halts 2 m before the crossing.  A genuinely stalled other
        # robot is skipped so the stall-escape arc still works.
        if self._row == "yield" and not self._yield_cleared():
            genuine_stall = bool(self._stall_since) and \
                (time.time() - self._stall_since >= 3.0)
            if not genuine_stall:
                if (self._robot_dist > self._yield_stop_dist and
                        self._own_speed > 0.05):
                    if self._last_action != "slow":
                        self._last_action = "slow"
                        self._last_reason = ("yield approach — decelerating, "
                                             "will stop at the yield point")
                        self._publish()
                elif self._last_action != "yield":
                    self._last_action = "yield"
                    self._last_reason = "yield — stopped at the yield point"
                    self._publish()
        sig = self._signature()
        changed = sig != self._last_signature
        self._last_signature = sig
        something = self._robot_close or self._human_det

        if not something:
            # Path clear -> no API call.  Default to proceed.
            if self._last_action != "proceed":
                self._last_action = "proceed"
                self._last_reason = "path clear"
                self._publish()
            return

        if not changed:
            return  # same situation -> keep last decision (no call)

        if time.time() - self._last_call_t < self._cooldown:
            return  # rate limit

        self._last_call_t = time.time()
        if self._busy:
            return
        self._busy = True
        threading.Thread(target=self._decide, daemon=True).start()

    def _situation(self):
        robot = (
            f"nearest robot: {self._robot_id or '?'} at {self._robot_dist:.2f} m, "
            f"bearing {self._robot_angle:+.0f} deg (negative=right, positive=left), "
            f"predicted closest approach (DCA) {self._robot_dca:.2f} m, "
            f"its speed: {self._other_speed:.2f} m/s "
            f"({'faster than me' if self._robot_faster else 'not faster than me'})"
        )
        a = abs(self._robot_angle)
        # A robot that has actually passed (beside/behind) -> resume.
        if self._robot_close and a > 60.0:
            if a > 100.0:
                robot += (
                    " — the robot is BEHIND me (already passed) — "
                    "do NOT yield, choose 'proceed'"
                )
            else:
                robot += (
                    " — the robot is BESIDE me (passing) — "
                    "do NOT yield, choose 'proceed'"
                )
        elif self._robot_close and a > 40.0 and self._range_rate > 0.05:
            # Passed our front and receding -> pass is over even before it
            # reaches 60 deg (90-deg crossing: robot_0 kept yielding while
            # robot_1 was already driving away, run_2026-08-24_14-10-25).
            robot += (
                " — the robot has PASSED my front and is moving AWAY "
                f"(range increasing {self._range_rate:+.2f} m/s) — the pass "
                "is over, do NOT yield, choose 'proceed'"
            )
        elif self._row == "yield":
            yield_s = (time.time() - self._yield_started_t
                       if self._yield_started_t else 0.0)
            stall_s = (time.time() - self._stall_since
                       if self._stall_since else 0.0)
            robot += (
                " — YOU are the YIELDING robot in this encounter (committed at "
                "first contact: the other robot was on YOUR right). Yield and "
                "let it pass; keep yielding until it is clearly past you "
                "(bearing > 60 deg, moving away, or gone). You have been yielding for "
                f"{yield_s:.1f} s and the other robot is "
                f"{'nearly stationary' if stall_s >= 1.0 else 'moving'} "
                f"(speed {self._other_speed:.2f} m/s, stalled {stall_s:.1f} s). "
                "If it is nearly stationary for 3+ seconds, the pass is STUCK "
                "— YOU (the yielding robot) must break it: stop yielding and "
                "choose arc_left/arc_right to go around it (pick the side with "
                "more free space, away from the other robot). The right-of-way "
                "robot will NOT move for you."
            )
        elif self._row == "way":
            robot += (
                " — YOU HAVE the right-of-way in this encounter (committed at "
                "first contact: the other robot was on YOUR left). Keep going — "
                "NEVER yield to it and NEVER escape around it on your own; the "
                "yielding robot will move aside. Even while it crosses or passes."
            )
        elif self._row == "headon" or (self._robot_close and a <= 10.0):
            robot += (
                " — TRUE HEAD-ON (robot dead ahead; DCA < 1 m is an imminent "
                "collision): pass with arc_left or arc_right, do NOT stop"
            )
        elif self._robot_close:
            robot += " — in front, off to the side (approaching/crossing)"
        human = (
            f"human: {'present' if self._human_det else 'none'}"
            + (f" at {self._human_dist:.2f} m, bearing {self._human_angle:+.0f} deg, "
               f"{'CLOSE (within 1.5 m)' if self._human_close else 'not close (>1.5 m)'}"
               if self._human_det else "")
        )
        motion = ""
        if self._human_det:
            rr, label = self._human_motion()
            motion = f", motion: {label} (range rate {rr:+.2f} m/s)"
        free = (f"free space: front {self._free_front:.1f} m, "
                f"left {self._free_left:.1f} m, right {self._free_right:.1f} m")
        return (
            f"Situation for robot {self.ns}:\n"
            f"- own speed: {self._own_speed:.2f} m/s\n"
            f"- {robot}\n"
            f"- {human}{motion}\n"
            f"- {free}\n"
            f"Decide the most socially appropriate action."
        )

    # --- LLM call ----------------------------------------------------------
    def _decide(self):
        try:
            action, reason, usage = self._call_llm()
        except Exception as e:      # network / parse failure -> safe fallback
            self.get_logger().warn(f"LLM call failed: {e} — keeping last action")
            action, reason, usage = self._last_action, "llm unavailable", None
        finally:
            self._busy = False

        if action not in ACTIONS:
            action = "proceed"

        # YIELDING-ROBOT ESCAPE GUARD: the yielding robot may ONLY move
        # (arc) to break the pass when the other robot is GENUINELY stalled
        # in front of it (slow + static bearing, >= 3 s). If the LLM tries to
        # arc while the other robot is still moving/passing, force "yield" —
        # moving early is what made robot_0 creep into the passing robot_1
        # (run_2026-08-24_13-40-36).
        if self._row == "yield":
            genuine_stall = bool(self._stall_since) and \
                (time.time() - self._stall_since >= 3.0)
            if action in ("arc_left", "arc_right") and not genuine_stall:
                action = "yield"
                reason = "staying yielded — the other robot is still passing"
            elif action not in ("arc_left", "arc_right") and genuine_stall:
                action = ("arc_left" if self._free_left >= self._free_right
                          else "arc_right")
                reason = "stall escape: other robot stalled in front, going around"

        # TWO-STAGE YIELD: keep a fresh LLM decision from undoing the
        # slow-approach — same mapping as the deterministic controller in _tick
        # (skip while genuinely stalled so the stall-escape arc survives).
        if (self._row == "yield" and not self._yield_cleared() and
                not (bool(self._stall_since) and
                     time.time() - self._stall_since >= 3.0)):
            if self._robot_dist > self._yield_stop_dist and self._own_speed > 0.05:
                action = "slow"
                reason = "yield approach — decelerating toward the yield point"

        # Deterministic yield release (LAST — overrides the LLM and the
        # guard): once the pass has clearly cleared, the yielding robot must
        # proceed even if the stall tracker still briefly sees a slow/receding
        # robot (run_2026-08-24_14-22-59: cleared at 46.6s, guard must not
        # turn that into a needless arc).
        if self._row == "yield" and self._yield_cleared():
            action = "proceed"
            reason = "pass cleared — resuming (deterministic release)"
            self._det_released = True
        # LATCHED release: even if the above check is momentarily False (e.g.
        # the range-rate is marginal right at 35-50 deg), once we have already
        # released this encounter a stale "yield" must not re-stop the robot
        # (run_2026-08-24_14-39-39: stale LLM yield at 41.8 s caused a stop-go).
        elif self._det_released and self._row == "yield":
            action = "proceed"
            reason = "pass cleared — already released (deterministic latch)"

        self._last_action = action
        self._last_reason = reason
        self._publish()

        if usage:
            tin, tout = usage
            total = tin + tout
            cost = tin * PRICE_IN + tout * PRICE_OUT
            self._calls += 1
            self._tokens_in += tin
            self._tokens_out += tout
            self._cost += cost
            self._write_usage_row(action, reason, tin, tout, total, cost)
            self.get_logger().info(
                f"LLM decision: {action} — {reason} "
                f"(in={tin} out={tout} tokens, ~${cost:.5f}, "
                f"run total ~${self._cost:.5f})")

    def _call_llm(self):
        if not self._key:
            return "proceed", "no api key", None
        body = {
            "model": self._model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": self._situation()},
            ],
            "max_tokens": self._max_tokens,
            "temperature": 0.0,
        }
        req = urllib.request.Request(
            "https://api.openai.com/v1/chat/completions",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json",
                     "Authorization": "Bearer " + self._key},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=30) as r:
            data = json.loads(r.read().decode())
        content = data["choices"][0]["message"]["content"]
        u = data.get("usage", {})
        action, reason = self._parse(content)
        usage = (u.get("prompt_tokens", 0), u.get("completion_tokens", 0))
        return action, reason, usage

    @staticmethod
    def _parse(content):
        """Extract {"action": ..., "reason": ...} tolerantly."""
        text = content.strip()
        if text.startswith("```"):
            text = text.strip("`")
            if text.startswith("json"):
                text = text[4:]
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            import re
            m = re.search(r'"action"\s*:\s*"([a-z]+)"', text)
            act = m.group(1) if m else "proceed"
            r = re.search(r'"reason"\s*:\s*"([^"]*)"', text)
            return act, (r.group(1) if r else "")
        return obj.get("action", "proceed"), obj.get("reason", "")

    def _write_usage_row(self, action, reason, tin, tout, total, cost):
        now = time.time()
        fcntl.flock(self._usage_file, fcntl.LOCK_EX)
        self._usage_writer.writerow([
            f"{now:.3f}", f"{now - self._start_time:.3f}", self.ns,
            action, reason, tin, tout, total, f"{cost:.6f}"])
        self._usage_file.flush()
        fcntl.flock(self._usage_file, fcntl.LOCK_UN)

    def write_summary(self):
        """Write per-run totals (calls / tokens / cost). Idempotent."""
        if self._summary_written:
            return
        self._summary_written = True
        try:
            with open(self._summary_path, "a", newline="") as f:
                w = csv.writer(f)
                if os.path.getsize(self._summary_path) == 0:
                    w.writerow(["robot_id", "calls", "input_tokens",
                                "output_tokens", "total_tokens", "cost_usd"])
                w.writerow([self.ns, self._calls, self._tokens_in,
                            self._tokens_out, self._tokens_in + self._tokens_out,
                            f"{self._cost:.6f}"])
        except OSError as e:
            self.get_logger().warn(f"could not write {self._summary_path}: {e}")
        if self._usage_file:
            try:
                self._usage_file.close()
            except Exception:
                pass
        self.get_logger().info(
            f"LLM usage summary: {self._calls} calls, "
            f"{self._tokens_in + self._tokens_out} tokens, "
            f"~${self._cost:.6f} -> {self._summary_path}")

    def _publish(self):
        a = String(); a.data = self._last_action
        r = String(); r.data = self._last_reason
        self._pub_action.publish(a)
        self._pub_reason.publish(r)
        # LLM soft speed cap (consumed by velocity_adaptor)
        if self._last_action == "slow":
            scale = 0.4
        elif self._last_action in ("arc_left", "arc_right"):
            scale = 0.3
        elif self._last_action in ("yield", "stop"):
            scale = 0.0
        else:
            scale = 1.0
        fs = Float32(); fs.data = scale
        self._pub_speed_scale.publish(fs)
        # Track how long we have been continuously yielding (for the stall
        # escape reported in the situation).
        if self._last_action == "yield":
            if self._yield_started_t == 0.0:
                self._yield_started_t = time.time()
        else:
            self._yield_started_t = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = LlmPlanner()
    stop = threading.Event()

    def _spin():
        rclpy.spin(node)

    spinner = threading.Thread(target=_spin, daemon=True)
    spinner.start()

    def _handle_signal(signum, _frame):
        node.get_logger().info(
            f"received signal {signum}; writing LLM usage summary")
        stop.set()

    # write the usage summary even on Ctrl+C (SIGINT) or kill (SIGTERM)
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        while not stop.is_set() and spinner.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    node.write_summary()
    node.destroy_node()
    rclpy.shutdown()
    spinner.join(timeout=2.0)


if __name__ == "__main__":
    main()
