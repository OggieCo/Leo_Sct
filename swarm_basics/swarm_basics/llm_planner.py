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
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from swarm_basics.run_utils import get_run_dir

MODEL = "gpt-4o-mini"
PRICE_IN = 0.15 / 1e6      # USD per input token (gpt-4o-mini)
PRICE_OUT = 0.60 / 1e6     # USD per output token

ACTIONS = ("proceed", "yield", "slow", "arc_left", "arc_right", "stop", "back")

SYSTEM_PROMPT = """You are the social decision layer of a mobile robot navigating around other robots and humans. Choose exactly ONE action:
- "proceed": continue at full speed along the current path (default when there is no conflict).
- "slow": keep driving but at reduced speed — use when someone is approaching but does NOT block you yet (anticipatory caution). The target speed is set separately (llm_speed_scale).
- "yield": stop and let the other pass. Yield to a robot ONLY when the situation report says YOU are the YIELDING robot in the encounter; also yield to a human that is CLOSE (within 1.5 m) and ahead of you or crossing your path. NEVER yield when you HAVE the right-of-way.  NOTE: for a head-on (robot dead ahead) do NOT yield — pass adaptively (see ADAPTIVE PASSING).
- "arc_left" / "arc_right": a smooth curved detour to that side, then continue toward the goal (NEVER an in-place spin). Use them to pass another robot when there is room — pick the side with MORE free space (see ADAPTIVE PASSING). NEVER arc when the other robot is too close to turn safely — use "back" instead.
- "stop": emergency stop (e.g., collision imminent).
- "back": reverse STRAIGHT backwards (no turning) ONLY to break a DEADLOCK — both robots are stuck (other robot speed < 0.1 m/s for 2+ s) and too close ahead (<= 1.2 m) with no other option. ONLY when the rear is clear (free space shows back >= 0.9 m). NEVER back while the other robot is still moving/passing — choose "stop" and let it finish. Once clear, resume navigation.
Rules: humans always have priority; prefer to wait for the other to pass and then continue straight (minimal deviation); never suggest in-place rotations.
ADAPTIVE PASSING (robot-robot): judge the geometry each tick and pick the SAFEST action:
- Robot dead ahead (|bearing| < 10 deg) but > 1.2 m away: pass with "arc_left" or "arc_right" — choose the side with MORE free space (compare free_left vs free_right; also steer away from walls and humans).
- Robot TOO CLOSE (<= 1.2 m ahead): DO NOT arc (a turn at close range swings you into it). If the other robot is STILL MOVING, hold with "stop"/"yield" and let it finish passing. Only if BOTH robots are stuck (deadlock, other speed < 0.1 m/s for 2+ s) reverse with "back".
- NEVER "proceed" toward a robot that is dead ahead and close; never arc into the side the other robot occupies. "yield" is only for side/crossing conflicts or humans.
A robot BESIDE or BEHIND you (bearing magnitude > 60 deg) has ALREADY passed — NEVER yield to it; choose "proceed".
RIGHT-OF-WAY IS LOCKED FOR THE WHOLE ENCOUNTER (right-hand traffic: the robot on the LEFT yields, the robot on the RIGHT has the way). The situation report tells you once, at first contact, whether YOU are the YIELDING robot or HAVE the right-of-way — follow it and do NOT change your mind mid-pass. If you HAVE the way: NEVER yield, even if the other robot is crossing, faster, or seems to be on your right as it passes. If you are YIELDING: stop and let it pass, then resume once it is clearly past (bearing > 60 deg).
STALL ESCAPE: ONLY the YIELDING robot may break a stuck pass. If you are the YIELDING robot and the other robot is nearly stationary (speed < 0.1 m/s) directly ahead for 5+ seconds, OR you have been yielding for 8+ seconds and it is STILL in front of you, the pass is STUCK — stop yielding and choose "arc_left"/"arc_right" to drive around it (prefer the side with more free space). A human near you means the other robot is probably paused for the human, NOT stuck — do NOT escape then. If you HAVE the right-of-way, NEVER escape on your own — keep your path; the yielding robot will move aside (the recovery layer handles it if it never does).
Use the motion/free-space hints: if a human is approaching (range rate negative) but still >1.5 m away, "slow" is often better than a full stop. A human reported "not close (>1.5 m)" is NOT a hard conflict — prefer "proceed" or "slow", not "yield".
A human that is STANDING STILL (range rate ~0) and OFF TO THE SIDE (bearing magnitude > 10 deg) is NOT a hard conflict — after a short observation the robot drives past it at reduced speed. Only yield to a human that is dead ahead or actively crossing your path.
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
        self._robot_opposition_deg = 180.0
        self._robot_faster = False
        self._robot_dist = float("inf")
        self._robot_id = ""
        self._human_det = False
        self._human_close = False
        self._human_dist = -1.0
        self._human_angle = 0.0
        self._human_gone_since = 0.0
        self._own_speed = 0.0
        self._other_speed = 0.0
        self._human_hist = []       # [(t, dist)] for range-rate
        self._human_ang_hist = []   # [(t, angle)] for crossing estimate
        self._free_front = float("inf")
        self._free_left = float("inf")
        self._free_right = float("inf")
        self._free_back = float("inf")

        # --- decision state ---
        self._last_signature = None
        self._last_call_t = 0.0
        self._last_action = "proceed"
        self._last_reason = "no conflict yet"
        self._busy = False
        self._row = None             
        self._row_clear_since = 0.0
        self._row_commits = {}       # robot_id -> (row, commit_time)
        self._row_commit_ttl = 3.0   
        self._row_resolve_grace = 2.0  
        self._row_committed_at = None  
        self._coord_note = ""       
        self._human_holds = {}       # robot_id -> last time it reported hold=True
        self._human_hold_ttl = 3.0   
        self._human_hold_grace = 2.0 
        self._det_released = False
        self._yield_stop_dist = 1.5
        self._back_dist = 1.2
        self._back_clear_dist = 0.7
        # Deadlock confirmation window (s): the other robot must have been
        # stalled ahead for this long AND we must be stopped too before the
        # robot is allowed to reverse.
        self._back_deadlock_s = 2.0
        self._human_slow_dist = 1.5
        self._human_stop_dist = 1.0      
        self._human_back_dist = 0.7       
        self._human_back_angle = 70.0    
        self._human_back_cool_s = 3.0     
        self._last_human_back_t = 0.0
        self._last_human_back_logt = 0.0  
        self._back_hold_s = 2.2
        self._back_until = 0.0
        self._human_pass_min_deg = 8.0    
        self._human_pass_max_deg = 85.0   
        self._human_pass_wait = 0.8       
        self._human_pass_lat = 0.3        
        self._human_pass_since = 0.0     
        self._row_front_deg = 30.0
        self._row_opp_deg = 30.0
        self._yield_started_t = 0.0  
        
        # How long the other robot has stalled ahead Mutual-block fallback (see
        # _commit_mutual_block): both robots stopped close to each other with no
        # concrete way/yield pair - the timestamp of when that condition began.
        self._mutual_block_since = 0.0
        self._yield_max_s = 8.0
        self._row_yield_since = 0.0  
        self._genuine_stall_s = 5.0
        self._rob_ang_prev = None    # previous other-robot bearing (stall check)
        self._rob_ang_prev_t = 0.0
        self._dist_prev = None       # previous other-robot range (range-rate)
        self._dist_prev_t = 0.0
        self._range_rate = 0.0       # + = other robot moving away

        # --- arc commit: ONE-SHOT sticky maneuver --------------------------
        # Arc owns the action until a DISTANCE-based clear, the hard cap, or
        # a human close; a cooldown blocks chained restarts.
        self._arc_action = ""            # "arc_left"/"arc_right" while committed
        self._arc_commit_start = 0.0     # when this maneuver was committed
        self._arc_until = 0.0            # ownership deadline (extendable)
        self._arc_hold_s = 4.5           # base ownership (arcs run ~3-5 s)
        self._arc_hard_s = 6.5           # absolute cap incl. extensions
        self._arc_min_s = 3.5            # no self-release before this (see _arc_cleared)
        self._arc_cool_until = 0.0       # no NEW arc before this time
        self._arc_cool_s = 2.0           # spacing between arc attempts
        self._arc_clear_since = 0.0      # debounce for the distance-based clear
        self._arc_clear_dist = 2.4       # m; pass considered over at/above this

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
        # Append-mode + lock: BOTH robots share this usage file.
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
        self.create_subscription(Float32, "robot_opposition_deg", self._cb_robot_opp, 10)
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
        # Shared coordination topic (GLOBAL namespace — every robot sees every
        # other robot's commitment, transient-local so the newest commit is
        # latched for late subscribers).
        row_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/social/row_commit",
                                 self._cb_row_commit, row_qos)
        # Shared human-hold topic (GLOBAL, transient-local like row_commit):
        # every robot reports whether it is stopped for a human, so a
        # yielding robot never stall-escapes into one that is only pausing.
        self.create_subscription(String, "/social/human_hold",
                                 self._cb_human_hold, row_qos)

        # --- publishers ---
        self._pub_row = self.create_publisher(String, "/social/row_commit", row_qos)
        self._pub_human_hold = self.create_publisher(String, "/social/human_hold", row_qos)
        self._pub_action = self.create_publisher(String, "llm_action", 10)
        self._pub_reason = self.create_publisher(String, "llm_reason", 10)
        self._pub_speed_scale = self.create_publisher(Float32, "llm_speed_scale", 10)
        # Maneuver speed floor for velocity_adaptor (0 = no floor): keeps an active arc
        # / right-of-way holder from being strangled by the robot proximity brake
        # (applied as max(robot_term, floor)).
        self._pub_floor = self.create_publisher(Float32, "social_speed_floor", 10)

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
    def _cb_robot_opp(self, m): self._robot_opposition_deg = float(m.data)
    def _cb_robot_faster(self, m): self._robot_faster = bool(m.data)
    def _cb_robot_dist(self, m): self._robot_dist = float(m.data)
    def _cb_robot_id(self, m): self._robot_id = m.data
    def _cb_human_det(self, m):
        self._human_det = bool(m.data)
        if not self._human_det:
            self._human_pass_since = 0.0   # restart the short wait next time
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
        front = left = right = back = float("inf")
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
            elif deg >= 150.0 or deg <= -150.0:
                back = min(back, r)
        self._free_front, self._free_left, self._free_right = front, left, right
        self._free_back = back

    def _back_clear(self):
        """True when the lidar rear sector is free enough to reverse safely."""
        return self._free_back >= self._back_clear_dist
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
    def _id_num(self, rid):
        """Numeric suffix of a robot id ('robot_3' -> 3) for the deterministic
        mutual-way tie-break; falls back to +inf (string ids sort nowhere)."""
        digits = "".join(ch for ch in str(rid) if ch.isdigit())
        try:
            return int(digits)
        except ValueError:
            return float("inf")

    def _publish_row(self):
        """Publish the current right-of-way commitment to the shared
        /social/row_commit topic so OTHER robots can coordinate."""
        msg = String()
        msg.data = json.dumps({
            "robot": self.ns,
            "row": self._row or "none",
            "t": time.time(),
        })
        self._pub_row.publish(msg)

    def _set_row(self, row):
        """Commit (or clear) the right-of-way role: record when it happened,
        reset per-encounter state, and publish the commitment to the shared
        /social/row_commit topic."""
        self._row = row
        self._row_committed_at = time.time() if row is not None else None
        if row == "yield":
            self._yield_started_t = time.time()
            self._row_yield_since = time.time()
            self._det_released = False
        else:
            self._row_yield_since = 0.0
        self._publish_row()

    def _cb_row_commit(self, m):
        """Track other robots' ROW commits.  Tricky race: both robots can
        commit 'way' in the same tick before seeing each other — the smaller
        numeric id re-resolves to 'yield' so the pair stays consistent."""
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        rid = d.get("robot")
        if not rid or rid == self.ns:
            return
        self._row_commits[rid] = (d.get("row"), float(d.get("t", 0.0)))
        now = time.time()
        for k in [k for k, (_, ct) in self._row_commits.items()
                  if now - ct > self._row_commit_ttl]:
            del self._row_commits[k]
        # Simultaneous mutual-way re-resolution (we are freshly committed
        # "way" and the other robot committed "way" too).
        if (self._row == "way" and d.get("row") == "way"
                and self._row_committed_at is not None
                and now - self._row_committed_at < self._row_resolve_grace
                and self._id_num(self.ns) < self._id_num(rid)):
            self._coord_note = f"mutual way with {rid} — smaller id yields"
            self._set_row("yield")
            self.get_logger().info(
                f'[{self.ns}] ROW re-resolved: {rid} also committed way '
                f'(mutual right-of-way) — smaller id yields')

    def _cb_human_hold(self, m):
        """Remember when the OTHER robot last reported it was holding for a
        human.  We keep the LAST hold-time (we do NOT delete on hold=False) so
        a robot that just resumed after a human pause still suppresses our
        escape for a short grace window while it ramps back up to speed."""
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        rid = d.get("robot")
        if not rid or rid == self.ns:
            return
        if d.get("hold"):
            self._human_holds[rid] = float(d.get("t", 0.0))
        now = time.time()
        for k in [k for k, ct in self._human_holds.items()
                  if now - ct > self._human_hold_ttl + self._human_hold_grace]:
            del self._human_holds[k]

    def _publish_human_hold(self):
        """Broadcast whether WE are currently stopped for a human (shared
        /social/human_hold).  The other robot uses this to avoid escaping into
        a robot that is only pausing for a human it may not see."""
        hold = bool(self._human_det) and not self._human_cleared()
        msg = String()
        msg.data = json.dumps({"robot": self.ns, "hold": hold, "t": time.time()})
        self._pub_human_hold.publish(msg)

    def _other_holding_human(self):
        """Is the OTHER robot paused for a human (from shared /social/human_hold
        reports)?  Includes a short grace window after its last report, so we
        keep waiting while it ramps back up after the human clears."""
        if not self._robot_id:
            return False
        t = self._human_holds.get(self._robot_id)
        return bool(t) and (time.time() - t <=
                            self._human_hold_ttl + self._human_hold_grace)

    def _resolve_row(self, local, other_id):
        """Coordination override for a NEW encounter: if the other robot has
        ALREADY published an active commitment, take the complementary role so
        two robots can never both commit 'way'.  Returns (row, note)."""
        if not other_id:
            return local, ""
        other = self._row_commits.get(other_id)
        if not other or time.time() - other[1] > self._row_commit_ttl:
            return local, ""
        other_row = other[0]
        if other_row == "headon":
            return "headon", f"{other_id} committed headon"
        if other_row == "yield":
            return "way", f"{other_id} is yielding to me"
        if other_row == "way":
            if local == "way":
                # Mutual right-of-way: deterministic tie-break (both robots
                # compute the same) — the smaller id yields.
                if self._id_num(self.ns) < self._id_num(other_id):
                    return "yield", f"{other_id} committed way, smaller id yields"
                return "way", f"{other_id} committed way, larger id keeps way"
            return local, ""
        return local, ""

    def _update_row(self):
        """Commit the right-of-way ONCE at first robot contact (like real
        traffic): re-evaluating left/right every tick makes the right-of-way
        robot doubt itself and freeze mid-pass.

        Coordination: consult the OTHER robot's published commitment on
        /social/row_commit first and take the complementary role."""
        now = time.time()
        if not self._robot_close:
            if self._row is not None:
                if self._row_clear_since == 0.0:
                    self._row_clear_since = now
                elif now - self._row_clear_since >= 2.0:
                    self._set_row(None)
                    self._row_clear_since = 0.0
                    self._yield_started_t = 0.0
                    self._stall_since = 0.0
                    self._det_released = False
                    self._coord_note = ""
            return
        self._row_clear_since = 0.0
        if self._row is None:
            # New encounter -> arm the deterministic release again.
            self._det_released = False
            a = abs(self._robot_angle)
            # Heading opposition: a low bearing is only a HEAD-ON when the headings are
            # actually opposed - a crossing robot passing in front also shows a low
            # bearing, but is NOT head-on.
            head_on_like = self._robot_opposition_deg <= self._row_opp_deg
            if head_on_like and a <= 10.0:
                local = "headon"   # true head-on: both arc
            elif self._robot_angle < 0:
                local = "yield"    # other on our right -> we yield
            elif head_on_like and a <= self._row_front_deg:
                # Mirror-symmetric oblique head-on (both see the other on their left):
                # both arc instead of both claiming "way" and deadlocking.
                local = "headon"
            else:
                local = "way"      # other clearly on our left -> the way
            final, note = self._resolve_row(local, self._robot_id)
            self._coord_note = note
            self._set_row(final)
            if note:
                self.get_logger().info(
                    f'[{self.ns}] ROW {final} (coordinated: {note})')

    def _commit_mutual_block(self):
        """Deterministic ROW assignment when a genuine MUTUAL BLOCK forms.

        If both robots sit stopped (< 0.05 m/s each) inside proximity range
        for >= 2 s and no concrete way/yield pair exists — nothing was
        committed, or the symmetric "headon" (both-arc) commit never unstuck
        the pair — assign the pair deterministically, with PRIORITY TO THE
        RIGHT:

          * other robot clearly on MY RIGHT (bearing < -band) -> I YIELD
            (the robot on the right has the way),
          * other robot clearly on MY LEFT  (bearing > +band) -> I get WAY
            (I am on the other robot's right),
          * head-on / mirror-symmetric geometry (both robots see the other
            on the same side) -> id tie-break: the smaller numeric id gets
            the way, so the pair always ends up split exactly one way /
            one yield.

        Peer commitments on /social/row_commit are respected first: a fresh
        "way" from the other robot makes us commit "yield" and vice versa;
        if BOTH ended up yielded, the smaller id switches to "way" so the
        pair cannot deadlock in a mutual yield (a simultaneous mutual WAY is
        still repaired by _cb_row_commit).  LLM-layer only - the pure-BT and
        stock-Nav2 conditions are untouched.
        """
        now = time.time()
        if (not self._robot_close or self._own_speed >= 0.05
                or self._other_speed >= 0.05):
            self._mutual_block_since = 0.0
            return
        if self._mutual_block_since == 0.0:
            self._mutual_block_since = now
            return
        if now - self._mutual_block_since < 2.0:
            return

        peer = self._row_commits.get(self._robot_id)
        peer_row = (peer[0] if peer and (now - peer[1]) <= self._row_commit_ttl
                    else None)
        a = self._robot_angle
        row = None
        why = ""
        if self._row == "way" and not (
                peer_row == "way"
                and self._id_num(self.ns) < self._id_num(self._robot_id)):
            return          # we already hold the way (alone)
        if self._row == "way":
            row, why = "yield", "mutual way - smaller id yields"
        elif peer_row == "way":
            if self._row == "yield":
                return      # already yielding to its way
            row, why = "yield", f"{self._robot_id} has the way (peer commit)"
        elif peer_row == "yield" and self._row != "yield":
            row, why = "way", f"{self._robot_id} yields (peer commit) - I proceed"
        elif peer_row == "yield" and self._row == "yield":
            if self._id_num(self.ns) < self._id_num(self._robot_id):
                row, why = "way", "both yielded - smaller id takes the way"
            else:
                return      # the smaller id breaks the mutual yield
        elif self._row == "yield":
            return          # we already yield; the timed escape handles a stall
        else:
            # row is None or a stalled "headon" and the peer is silent ->
            # geometry rule (priority to the right).
            head_on_like = self._robot_opposition_deg <= self._row_opp_deg
            if head_on_like or abs(a) <= self._row_front_deg:
                row = ("way"
                       if self._id_num(self.ns) < self._id_num(self._robot_id)
                       else "yield")
                why = (f"head-on/mirror geometry ({a:+.0f} deg) - "
                       "smaller id gets the way")
            elif a < 0.0:
                row, why = "yield", (f"other on my right ({a:+.0f} deg) - "
                                     "give way to the right")
            else:
                row, why = "way", (f"other on my left ({a:+.0f} deg) - "
                                   "I am on its right, keep the way")
        if row is None:
            return
        stalled = now - self._mutual_block_since
        self._coord_note = f"mutual block {stalled:.1f}s - {why}"
        self._set_row(row)
        self.get_logger().info(
            f'[{self.ns}] ROW {row} (mutual-block fallback: {why})')

    def _update_stall(self):
        """Track how long the other robot has been GENUINELY stalled ahead:
        low speed AND static bearing.  A robot merely passing slowly (e.g.
        mid-arc) has a moving bearing and is NOT stalled — otherwise the
        yielding robot escapes into a mid-maneuver peer and they tangle."""
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

    def _yield_stuck(self):
        """Is the pass stuck from OUR (yielding) side, so we may break it
        with an arc?  True when EITHER
          (a) the other robot is genuinely stalled ahead (slow + static
              bearing for _genuine_stall_s) and no human is interfering, OR
          (b) we have held the yield role for _yield_max_s and it is still
              in front — a robot creeping just fast enough to evade the
              stall detector must not block us forever.

        HUMAN GATE: a nearby human may mean the other robot is PAUSED for it
        and will clear on its own — do not arc into it.  The gate blocks only
        the short escape (a); the timed backstop (b) is never gated, so an
        unrelated freeze cannot trap us.  Mirrors the reactive tree's
        RobotYieldMax."""
        now = time.time()
        human_gate = (self._human_det and not self._human_cleared()) or \
            self._other_holding_human()
        genuine_stall = (not human_gate) and bool(self._stall_since) and \
            (now - self._stall_since >= self._genuine_stall_s)
        timed = (self._row_yield_since > 0.0
                 and now - self._row_yield_since >= self._yield_max_s
                 and self._robot_close and abs(self._robot_angle) <= 60.0)
        return genuine_stall or timed

    def _yield_cleared(self):
        """Deterministic yield release (bypasses the LLM and its cooldown):
        the pass is over once the other robot leaves our front corridor —
        bearing past ~50 deg, OR receding (range-rate > 0) past ~0.8 m
        lateral.  Without this the yielding robot hangs on for seconds after
        the other has driven away.  Used in _tick and _decide."""
        a = abs(self._robot_angle)
        if a > 50.0:
            # Bearing alone is not enough: at 50+ deg the peer can still be
            # only ~0.6 m away laterally — resume only once it is beyond the
            # corridor (distance or lateral margin).
            lat = self._robot_dist * math.sin(math.radians(self._robot_angle))
            return self._robot_dist > 1.0 or abs(lat) > 0.8
        if a <= 35.0:
            return False
        lat = self._robot_dist * math.sin(math.radians(self._robot_angle))
        return self._range_rate > 0.05 and abs(lat) > 0.8

    # --- arc commit (one-shot sticky maneuver) -----------------------------
    def _arc_active(self):
        """Own an arc maneuver right now (action locked to arc_*)?"""
        return bool(self._arc_action) and time.time() < self._arc_until

    def _start_arc_commit(self, action):
        """Take ownership of a smooth-avoidance arc (called wherever an arc
        action is chosen).  Returns False when the post-arc cooldown blocks
        the attempt, so callers fall back to their non-arc branch instead of
        chaining restarts (one-shot semantics)."""
        now = time.time()
        if now < self._arc_cool_until:
            return False
        # Tricky: re-committing the SAME action keeps its original start
        # time, so renewals cannot push the hard cap out indefinitely.
        if not self._arc_active() or self._arc_action != action:
            self._arc_commit_start = now
        self._arc_action = action
        self._arc_until = min(now + self._arc_hold_s,
                              self._arc_commit_start + self._arc_hard_s)
        self._arc_clear_since = 0.0
        return True

    def _end_arc_commit(self, cool_s=None):
        """Release the arc ownership and start the post-arc cooldown."""
        if self._arc_action:
            held = time.time() - self._arc_commit_start
            self.get_logger().info(
                f"[{self.ns}] arc commit {self._arc_action} finished "
                f"(owned {held:.1f} s)")
        self._arc_action = ""
        self._arc_until = 0.0
        self._arc_cool_until = time.time() + (
            self._arc_cool_s if cool_s is None else cool_s)
        self._arc_clear_since = 0.0

    def _arc_cleared(self):
        """Distance-based pass-clear test for a committed arc.

        Tricky: BEARING is not usable while WE turn — it sweeps past 50 deg
        mid-maneuver, _yield_cleared() reads "cleared", the BT halts Arc,
        the robot straightens, the bearing shrinks, Arc restarts — a
        sub-second limit cycle.  Distance is the only honest signal.
        A human coming close also ends the maneuver (safety first)."""
        if self._human_close:
            return True
        # MIN OWNERSHIP: a distance-clear may not release the maneuver until
        # it has had time to execute (an arc takes ~2.4 s + start latency);
        # the hard cap and a human still end it regardless.
        if time.time() - self._arc_commit_start < self._arc_min_s:
            return False
        cleared = (self._robot_dist > self._arc_clear_dist or
                   (not self._robot_close and self._robot_dist > 1.8))
        now = time.time()
        if cleared:
            if self._arc_clear_since == 0.0:
                self._arc_clear_since = now
            return now - self._arc_clear_since >= 0.5
        self._arc_clear_since = 0.0
        return False

    def _arc_tick_update(self):
        """Per-tick arc-commit housekeeping: distance-release, extension
        while the encounter is still unresolved, hard-cap expiry, and the
        maneuver speed floor broadcast."""
        now = time.time()
        if self._arc_action:
            if self._arc_cleared():
                if self._human_close:
                    self._end_arc_commit()          # safety abort
                    if self._last_action in ("arc_left", "arc_right"):
                        self._last_action = "stop"
                        self._last_reason = ("human close — arc maneuver "
                                             "aborted, holding")
                        self._publish()
                else:
                    self._end_arc_commit(cool_s=1.0)  # cleared -> quick resume
                    if self._last_action != "proceed":
                        self._last_action = "proceed"
                        self._last_reason = "arc complete — pass cleared"
                        self._det_released = True
                        self._publish()
            elif now >= self._arc_until:
                if now < self._arc_commit_start + self._arc_hard_s:
                    # still blocked -> keep turning (never past the hard cap)
                    self._arc_until = min(now + 1.0,
                                          self._arc_commit_start + self._arc_hard_s)
                else:
                    self._end_arc_commit()          # hard cap — hand back control
        # SPEED FLOOR (velocity_adaptor, 0 = no floor): a maneuver needs motion to
        # produce separation, and the right-of-way holder must not be strangled by the
        # proximity brake while its peer yields.
        floor = 0.0
        if self._arc_active():
            floor = 0.55
        elif self._row == "way":
            peer = self._row_commits.get(self._robot_id)
            if (peer and peer[0] == "yield" and
                    now - peer[1] <= self._row_commit_ttl and
                    self._robot_dist > 1.0 and abs(self._robot_angle) <= 45.0):
                floor = 0.55
        fs = Float32()
        fs.data = float(floor)
        self._pub_floor.publish(fs)

    def _human_cleared(self):
        """Deterministic human release — adaptive to ANY human geometry:
        the robot resumes once the human is genuinely OUT of its forward
        corridor, so the LLM's endless 'slow' can never freeze it there.
        Release if ANY:
          (a) not detected for a 1.5 s debounce (noisy detector / out of view),
          (b) beside/behind the robot (bearing magnitude > 80 deg),
          (c) > 0.9 m laterally off the heading line and not actively
              closing — it has passed its closest approach.
        """
        if not self._human_det:
            if self._human_gone_since == 0.0:
                self._human_gone_since = time.time()
            # Debounce: a short detection dropout must not release the hold
            # (the rover would accelerate into the person it just lost).
            return time.time() - self._human_gone_since >= 1.5
        if self._human_dist < 0.0:
            return True                      # no range -> no human (ghost guard)
        self._human_gone_since = 0.0
        if abs(self._human_angle) > 80.0:
            return True                       # human beside/behind us
        rr, _ = self._human_motion()
        # A human beyond the slowdown range that is not actually closing is
        # NOT a conflict (noisy depth can report a far person as close) —
        # hold only when close (human_close <= 1.5 m) or truly approaching.
        if (not self._human_close and
                (self._human_dist > self._human_slow_dist or rr >= -0.05)):
            return True
        lat = self._human_dist * math.sin(math.radians(self._human_angle))
        return abs(lat) > 0.9 and rr > -0.1

    def _human_passable(self):
        """Can we drive AROUND this human instead of holding?

        True only when the human is (a) off to the side (bearing outside the
        dead-ahead cone but not yet beside/behind), (b) NOT actively closing
        on our path (stationary or moving away — never cut in front of a
        crossing human), (c) leaving enough lateral room to slip past, and
        (d) has been in that state for the short grace wait.  Returns False
        (and resets the wait) the moment any gate fails, so a stationary
        human that briefly moves back toward us goes back to a proper yield.
        """
        a = abs(self._human_angle)
        if a < self._human_pass_min_deg or a > self._human_pass_max_deg:
            self._human_pass_since = 0.0
            return False
        rr, _ = self._human_motion()
        if rr < -0.05:
            self._human_pass_since = 0.0
            return False
        lat = self._human_dist * math.sin(math.radians(self._human_angle))
        if lat < self._human_pass_lat:
            self._human_pass_since = 0.0
            return False
        if self._human_pass_since == 0.0:
            self._human_pass_since = time.time()
        return time.time() - self._human_pass_since >= self._human_pass_wait

    # --- event-driven tick -------------------------------------------------
    def _signature(self):
        # Coarse rounding on purpose: sensor noise must not re-trigger calls.
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
        self._commit_mutual_block()
        self._update_stall()
        # Share our own human-hold state (4 Hz, cheap): tells the OTHER robot
        # we are stopped for a human so it must not stall-escape into us.
        self._publish_human_hold()
        # Arc-commit housekeeping + the maneuver speed floor broadcast.
        self._arc_tick_update()
        # Deterministic release: don't wait for the LLM/cooldown once the
        # pass has clearly cleared.
        if (self._row == "yield" and self._yield_cleared() and
                not self._arc_active() and
                self._last_action != "proceed"):
            self._last_action = "proceed"
            self._last_reason = "pass cleared — resuming (deterministic release)"
            self._det_released = True
            self._publish()
        # TWO-STAGE YIELD (deterministic, every tick): while the other robot is still
        # farther than _yield_stop_dist and we are still moving, publish "slow"
        # (decelerate + creep closer) instead of a full stop.
        if (self._row == "yield" and not self._yield_cleared() and
                not self._arc_active()):
            if self._yield_stuck():
                # FORCED ESCAPE: once stuck (genuine stall OR _yield_max_s elapsed with
                # the peer still in front), publish the arc directly - never let the
                # LLM/guard flip it back to yield (that livelocks the pair for minutes).
                if self._last_action not in ("arc_left", "arc_right"):
                    cand = ("arc_left" if self._free_left >= self._free_right
                            else "arc_right")
                    # One-shot: the cooldown gate blocks chained restarts
                    # (see the arc-commit comment in __init__).
                    if self._start_arc_commit(cand):
                        self._last_action = cand
                        self._last_reason = ("stall escape: other robot still "
                                             "blocking after yield, going around")
                        self._publish()
            else:
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
        # NO ROBOT SLOWDOWN BEYOND +/-60 deg: the adaptor ignores rovers beside/behind
        # (|bearing| > 60), but an LLM "slow" would still pay a tax for them.
        if (self._last_action == "slow" and self._robot_close and
                abs(self._robot_angle) > 60.0 and
                not (self._human_det and not self._human_cleared())):
            self._last_action = "proceed"
            self._last_reason = ("robot beside/behind (>60 deg) — "
                                 "resuming full speed")
            self._publish()
        # HUMAN deterministic hold/pass/release (the LLM alone dithers in 'slow' with no
        # resolution).
        if (self._human_det and self._row != "yield"
                and not self._arc_active()):
            if self._human_cleared():
                # Release "stop" as well: an LLM "stop" for a human that has
                # cleared would otherwise park the rover until the next LLM
                # call (~2.5 s) — unless a robot is actually blocking.
                blocked = (self._robot_close
                           and abs(self._robot_angle) <= 60.0)
                if (self._last_action in ("slow", "yield", "stop")
                        and not blocked):
                    self._last_action = "proceed"
                    self._last_reason = ("human has cleared — resuming "
                                         "(deterministic release)")
                    self._publish()
            elif self._human_passable():
                if self._last_action != "slow":
                    self._last_action = "slow"
                    self._last_reason = ("human stationary off to the side — "
                                         "passing around it")
                    self._publish()
            else:
                if (self._human_dist > self._human_stop_dist and
                        self._own_speed > 0.05):
                    if self._last_action != "slow":
                        self._last_action = "slow"
                        self._last_reason = ("human ahead — decelerating, "
                                             "will hold until it clears")
                        self._publish()
                elif self._last_action != "yield":
                    self._last_action = "yield"
                    self._last_reason = ("human close ahead — holding "
                                         "until it clears")
                    self._publish()
        # HUMAN BACK-OFF (independent of the human-hold branch and the robot ROW state):
        # a person inside _human_back_dist is dangerously close - retreat via the BT
        # "back" action as soon as the rear is clear.
        if (self._human_det and not self._arc_active() and
                0.0 <= self._human_dist <= self._human_back_dist and
                abs(self._human_angle) <= self._human_back_angle):
            now_b = time.time()
            if not self._back_clear():
                if now_b - self._last_human_back_logt >= 2.0:
                    self._last_human_back_logt = now_b
                    self.get_logger().info(
                        f"[{self.ns}] human at {self._human_dist:.2f} m — "
                        "back-off blocked (rear not clear)")
            elif now_b - self._last_human_back_t >= self._human_back_cool_s:
                self._last_human_back_t = now_b
                self._back_until = now_b + self._back_hold_s
                if self._last_action != "back":
                    self._last_action = "back"
                    self._last_reason = ("human dangerously close — "
                                         "backing off to give space")
                    self._publish()
        # DEADLOCK BACK-UP (deterministic, LAST): reverse only on a GENUINE deadlock -
        # peer stalled ahead for >= _back_deadlock_s AND we are stopped too.
        if (self._robot_close and self._robot_dist <= self._back_dist and
                abs(self._robot_angle) <= 60.0 and not self._arc_active()):
            now = time.time()
            deadlock = (self._own_speed < 0.1 and
                        bool(self._stall_since) and
                        now - self._stall_since >= self._back_deadlock_s)
            if deadlock and self._back_clear():
                if self._last_action != "back":
                    self._last_action = "back"
                    self._last_reason = ("deadlock — both robots stuck, "
                                         "backing up to separate")
                    self._publish()
            elif self._last_action != "stop":
                self._last_action = "stop"
                self._last_reason = ("robot too close ahead — holding "
                                     "(back only for deadlocks)")
                self._publish()
        # BACK COMMIT (last word in the tick): while a human back-off is in
        # flight, nothing else may cancel the retreat — the BT halts BackUp
        # the moment the action changes.
        if time.time() < self._back_until and self._last_action != "back":
            self._last_action = "back"
            self._last_reason = "human back-off — completing the retreat"
            self._publish()
        sig = self._signature()
        changed = sig != self._last_signature
        self._last_signature = sig
        something = self._robot_close or self._human_det

        # ARC COMMIT: no LLM call while our own maneuver is executing and the pass has
        # not cleared - the action is already decided (finish the turn) and a fresh
        # "stop"/"yield" answer is exactly what used to cancel it mid-maneuver.
        if self._arc_active() and not self._arc_cleared():
            return

        if not something:
            # Path clear -> no API call.  Default to proceed.
            if self._last_action != "proceed":
                self._last_action = "proceed"
                self._last_reason = "path clear"
                self._publish()
            return

        if not changed:
            return  #keep last decision (no call)

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
            # Passed our front and receding -> the pass is over even before
            # the bearing reaches 60 deg.
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
            origin = ("; coordinated: " + self._coord_note
                      if self._coord_note else "")
            robot += (
                " — YOU are the YIELDING robot in this encounter (committed at "
                "first contact" + origin + "). Yield and "
                "let it pass; keep yielding until it is clearly past you "
                "(bearing > 60 deg, moving away, or gone). You have been yielding for "
                f"{yield_s:.1f} s and the other robot is "
                f"{'nearly stationary' if stall_s >= 1.0 else 'moving'} "
                f"(speed {self._other_speed:.2f} m/s, stalled {stall_s:.1f} s). "
                "If it is nearly stationary for 5+ seconds, or you have been "
                "yielding for 8+ seconds and it is STILL in front, the pass is "
                "STUCK — YOU (the yielding robot) must break it: stop yielding "
                "and choose arc_left/arc_right to go around it (pick the side "
                "with more free space, away from the other robot). But if a "
                "human is present and not cleared, the other robot is likely "
                "paused for it — wait, do NOT escape. The right-of-way robot "
                "will NOT move for you."
            )
        elif self._row == "way":
            origin = ("; coordinated: " + self._coord_note
                      if self._coord_note else "")
            robot += (
                " — YOU HAVE the right-of-way in this encounter (committed at "
                "first contact" + origin + "). Keep going — "
                "NEVER yield to it and NEVER escape around it on your own; the "
                "yielding robot will move aside. Even while it crosses or passes."
            )
        elif self._row == "headon" or (self._robot_close and a <= 10.0):
            robot += (
                " — TRUE HEAD-ON (robot dead ahead; DCA < 1 m is an imminent "
                "collision): pass adaptively — arc to the side with more "
                "free space if there is room (> 1.2 m); if it is TOO CLOSE "
                "(<= 1.2 m) hold ('stop') and let it pass — only if BOTH "
                "robots are stuck (deadlock) reverse with 'back'"
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
                f"left {self._free_left:.1f} m, right {self._free_right:.1f} m, "
                f"back {self._free_back:.1f} m")
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

        # YIELDING-ROBOT ESCAPE GUARD: the yielding robot may only arc when
        # the peer is genuinely stalled in front; arcing while it is still
        # moving/passing creeps straight into it — force "yield" instead.
        if self._row == "yield":
            stuck = self._yield_stuck()
            if action in ("arc_left", "arc_right") and not stuck:
                action = "yield"
                reason = "staying yielded — the other robot is still passing"
            elif action not in ("arc_left", "arc_right") and stuck:
                cand = ("arc_left" if self._free_left >= self._free_right
                        else "arc_right")
                if self._start_arc_commit(cand):
                    action = cand
                    reason = ("stall escape: other robot still blocking after "
                              "yield, going around")
                else:
                    action = "yield"
                    reason = "staying yielded — arc maneuver cooling down"

        # TWO-STAGE YIELD: keep a fresh LLM decision from undoing the
        # slow-approach — same mapping as the deterministic controller in _tick
        # (skip while genuinely stalled so the stall-escape arc survives).
        if (self._row == "yield" and not self._yield_cleared() and
                not self._yield_stuck()):
            if self._robot_dist > self._yield_stop_dist and self._own_speed > 0.05:
                action = "slow"
                reason = "yield approach — decelerating toward the yield point"

        # Deterministic yield release (overrides the LLM and the guard):
        # once the pass has cleared, proceed — the guard must not turn a
        # slow/receding peer into a needless arc.
        if self._row == "yield" and self._yield_cleared():
            action = "proceed"
            reason = "pass cleared — resuming (deterministic release)"
            self._det_released = True
        # LATCHED release: the check above can flicker False (range-rate is marginal at
        # 35-50 deg); once released, a stale "yield" must not re-stop the robot.
        elif self._det_released and self._row == "yield":
            action = "proceed"
            reason = "pass cleared — already released (deterministic latch)"

        # HUMAN deterministic layer (overrides the LLM's human dithering: repeated
        # "slow" with no resolution can freeze the robot on the human's line).
        if self._human_det and self._row != "yield":
            if self._human_cleared():
                # Same as the tick path: a cleared human releases "stop" too,
                # unless a robot is genuinely blocking ahead.
                blocked = (self._robot_close
                           and abs(self._robot_angle) <= 60.0)
                if action in ("slow", "yield", "stop") and not blocked:
                    action = "proceed"
                    reason = ("human has cleared — resuming "
                              "(deterministic release)")
            elif self._human_passable():
                action = "slow"
                reason = ("human stationary off to the side — "
                          "passing around it")
            else:
                if self._human_dist > self._human_stop_dist and \
                        self._own_speed > 0.05:
                    action = "slow"
                    reason = "human ahead — decelerating toward the hold point"
                else:
                    action = "yield"
                    reason = "human close ahead — holding until it clears"

        # HUMAN BACK-OFF (mirror of the tick path; runs even while a robot ROW role is
        # active - a person at arm's length outranks politeness to a rover).
        if (self._human_det and not self._arc_active() and
                0.0 <= self._human_dist <= self._human_back_dist and
                abs(self._human_angle) <= self._human_back_angle and
                self._back_clear() and
                time.time() - self._last_human_back_t >=
                self._human_back_cool_s):
            self._last_human_back_t = time.time()
            self._back_until = self._last_human_back_t + self._back_hold_s
            action = "back"
            reason = "human dangerously close — backing off to give space"

        # NO ROBOT SLOWDOWN BEYOND +/-60 deg (see the tick path): a rover
        # beside/behind never keeps us slowed down — a human still can.
        if (action == "slow" and self._robot_close and
                abs(self._robot_angle) > 60.0 and
                not (self._human_det and not self._human_cleared())):
            action = "proceed"
            reason = "robot beside/behind (>60 deg) — no slowdown"

        # DEADLOCK BACK-UP (deterministic, LAST — overrides the LLM and the
        # human layer): reverse only on a genuine deadlock; otherwise hold.
        if (self._robot_close and self._robot_dist <= self._back_dist and
                abs(self._robot_angle) <= 60.0):
            now = time.time()
            deadlock = (self._own_speed < 0.1 and
                        bool(self._stall_since) and
                        now - self._stall_since >= self._back_deadlock_s)
            if deadlock and self._back_clear():
                action = "back"
                reason = "deadlock — both robots stuck, backing up to separate"
            else:
                action = "stop"
                reason = "robot too close ahead — holding (back only for deadlocks)"

        # ARC COMMIT: a committed one-shot arc owns the action until the pass is
        # genuinely over, the hard cap elapses, or a human comes close - otherwise the
        # hold layer or a fresh LLM "stop" cancels it ~1 s in.
        if self._arc_active() and not self._arc_cleared():
            if self._human_close:
                self._end_arc_commit()
            else:
                action = self._arc_action
                reason = "arc commit — finishing the maneuver (pass not cleared)"
        elif action in ("arc_left", "arc_right"):
            if self._start_arc_commit(action):
                reason = f"{reason} [arc committed — will run to completion]"
            else:
                action = "proceed"
                reason = "arc cooling down — proceeding while the pass settles"

        # BACK COMMIT (final): a human back-off in flight is not cancelable by
        # the guards above either (see the tick path).
        if time.time() < self._back_until:
            action = "back"
            reason = "human back-off — completing the retreat"

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
        # LLM soft speed cap (consumed by velocity_adaptor).  "slow" is a
        # cautious creep, not a near-stop; hard stops (yield/stop = 0.0) and
        # the adaptor's own zero are unchanged.
        if self._last_action == "slow":
            scale = 0.65
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
