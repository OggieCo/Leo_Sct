#!/usr/bin/env python3
"""llm_planner — LLM-based high-level decision layer for social navigation.

Subscribes to the perception topics (robot proximity + human detection) and
calls OpenAI (gpt-4o-mini) ONLY when the situation materially changes and
something actually needs deciding (event-driven).  Publishes:

  llm_action   (std_msgs/String)  yield | proceed | arc | stop
  llm_reason   (std_msgs/String)  one-line natural-language justification

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
from std_msgs.msg import Bool, Float32, String

from swarm_basics.run_utils import get_run_dir

MODEL = "gpt-4o-mini"
PRICE_IN = 0.15 / 1e6      # USD per input token (gpt-4o-mini)
PRICE_OUT = 0.60 / 1e6     # USD per output token

ACTIONS = ("proceed", "yield", "arc", "stop")

SYSTEM_PROMPT = """You are the social decision layer of a mobile robot navigating around other robots and humans. Choose exactly ONE action:
- "proceed": continue along the current path (the default when there is no conflict).
- "yield": stop and let the other pass. Yield when: (a) head-on with another robot, (b) the other robot is on YOUR RIGHT (right-hand traffic) and not clearly slower, (c) the other robot is FASTER than you, (d) a human is close and ahead of you or crossing your path.
- "arc": after having yielded long enough and the other is STILL blocking, drive a smooth curved detour around them. NEVER an in-place spin.
- "stop": emergency stop (e.g., collision imminent).
Rules: humans always have priority; prefer to wait for the other to pass and then continue straight (minimal deviation); never suggest in-place rotations.
Respond with a single JSON object, no markdown, no extra text: {"action": "yield|proceed|arc|stop", "reason": "short reason"}"""


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
        self._human_dist = -1.0
        self._human_angle = 0.0
        self._own_speed = 0.0

        # --- decision state ---
        self._last_signature = None
        self._last_call_t = 0.0
        self._last_action = "proceed"
        self._last_reason = "no conflict yet"
        self._busy = False

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
        self._usage_file = open(self._usage_path, "w", newline="")
        self._usage_writer = csv.writer(self._usage_file)
        self._usage_writer.writerow([
            "timestamp", "elapsed_sec", "robot_id", "action", "reason",
            "in_tokens", "out_tokens", "total_tokens", "cost_usd"])
        self._usage_file.flush()

        # --- subscriptions ---
        self.create_subscription(Bool, "robot_close", self._cb_robot_close, 10)
        self.create_subscription(Float32, "robot_angle", self._cb_robot_angle, 10)
        self.create_subscription(Float32, "robot_dca", self._cb_robot_dca, 10)
        self.create_subscription(Bool, "robot_faster", self._cb_robot_faster, 10)
        self.create_subscription(Float32, "nearest_robot_dist", self._cb_robot_dist, 10)
        self.create_subscription(String, "nearest_robot_id", self._cb_robot_id, 10)
        self.create_subscription(Bool, "human_detected", self._cb_human_det, 10)
        self.create_subscription(Float32, "human_distance", self._cb_human_dist, 10)
        self.create_subscription(Float32, "human_angle", self._cb_human_angle, 10)
        self.create_subscription(Odometry, "odom", self._cb_odom, 10)

        # --- publishers ---
        self._pub_action = self.create_publisher(String, "llm_action", 10)
        self._pub_reason = self.create_publisher(String, "llm_reason", 10)

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
    def _cb_human_dist(self, m): self._human_dist = float(m.data)
    def _cb_human_angle(self, m): self._human_angle = float(m.data)
    def _cb_odom(self, m): self._own_speed = m.twist.twist.linear.x

    # --- event-driven tick -------------------------------------------------
    def _signature(self):
        return (
            round(self._robot_dist, 1), round(self._robot_angle, 5),
            round(self._robot_dca, 1), self._robot_faster, self._robot_close,
            self._robot_id,
            self._human_det, round(self._human_dist, 1),
            round(self._human_angle, 5),
        )

    def _tick(self):
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
            f"is it faster than me: {'yes' if self._robot_faster else 'no'}"
        )
        human = (
            f"human: {'present' if self._human_det else 'none'}"
            + (f" at {self._human_dist:.2f} m, bearing {self._human_angle:+.0f} deg" if self._human_det else "")
        )
        return (
            f"Situation for robot {self.ns}:\n"
            f"- own speed: {self._own_speed:.2f} m/s\n"
            f"- {robot}\n"
            f"- {human}\n"
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
        self._usage_writer.writerow([
            f"{now:.3f}", f"{now - self._start_time:.3f}", self.ns,
            action, reason, tin, tout, total, f"{cost:.6f}"])
        self._usage_file.flush()

    def write_summary(self):
        """Write per-run totals (calls / tokens / cost). Idempotent."""
        if self._summary_written:
            return
        self._summary_written = True
        try:
            with open(self._summary_path, "w", newline="") as f:
                w = csv.writer(f)
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
