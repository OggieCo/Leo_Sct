#!/usr/bin/env python3
"""yolo_human_processor — YOLO person detection + depth ranging.

Drop-in replacement for image_human_processor (the depth-heuristic detector):
publishes the SAME topics (human_detected / human_close / human_distance /
human_angle / human_info / wall_in_view) so random_walk, the IsHumanClose BT
plugin and Nav2 work completely unchanged.

Pipeline:
  RGB image  -> YOLOv8n (COCO class 0 = person) -> best person box
  depth image -> distance at the centre of the person's box
  publishes distance / angle / close flags.

Unlike the depth-blob heuristic this:
  * never confuses a wall for a person (trained on real photos),
  * detects the person continuously at any range (no tuning gap),
  * gives a bounding box (usable later by the LLM scene-description step).
"""

import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


class YoloHumanProcessor(Node):
    def __init__(self):
        super().__init__('yolo_human_processor')
        self.declare_parameter('close_distance', 1.5)
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('hfov', 1.51844)
        self.declare_parameter('detection_hold_s', 1.0)
        # Max inference-rate control (2026-09-18 load fix): at 5 Hz per robot
        # × 5 robots the YOLO workers saturated the CPU and starved SLAM/Nav2
        # bringup.  0.33 s ≈ 3 Hz is plenty for human proximity at rover
        # speeds.  Set 0.0 to disable the throttle (infer on every frame).
        self.declare_parameter('process_period_sec', 0.33)

        # --- compute placement -------------------------------------------
        # Env-overridable so the launch / parallel harness can tune a run
        # WITHOUT editing code (each robot gets its own process, so these are
        # per-robot settings):
        #   YOLO_DEVICE=cuda|0|cpu   run inference on the GPU (RTX 3050 Ti)
        #   YOLO_TORCH_THREADS=N     cap torch CPU threads per worker.  The
        #                            default is 8 = nproc/2, so five workers
        #                            would demand 40 threads on a 16-thread
        #                            box and thrash (the main 5-rover FPS
        #                            killer after per-frame inference).
        #   YOLO_PERIOD=0.33         inference period (s) -> ~3 Hz
        #   YOLO_HALF=1              FP16 inference (GPU only)
        self.declare_parameter('device', os.environ.get('YOLO_DEVICE', 'cpu'))
        self.declare_parameter(
            'torch_threads', int(os.environ.get('YOLO_TORCH_THREADS', '2')))
        self.declare_parameter('half', os.environ.get('YOLO_HALF', '0') == '1')

        self.close_dist_ = self.get_parameter('close_distance').value
        self.conf_ = self.get_parameter('confidence').value
        self.model_path_ = self.get_parameter('model_path').value
        self.input_size_ = self.get_parameter('input_size').value
        self.hfov_ = self.get_parameter('hfov').value
        self.hold_s_ = self.get_parameter('detection_hold_s').value
        self.process_period_ = float(
            os.environ.get('YOLO_PERIOD')
            or self.get_parameter('process_period_sec').value)
        self.device_ = str(self.get_parameter('device').value)
        self.torch_threads_ = int(self.get_parameter('torch_threads').value)
        self.half_ = bool(self.get_parameter('half').value)
        if self.device_.strip().lower() == 'cpu':
            self.half_ = False   # torch half precision needs CUDA
        self.last_infer_ = 0.0

        # --- publishers (same names as image_human_processor) ---
        self.det_pub_ = self.create_publisher(Bool, 'human_detected', 10)
        self.close_pub_ = self.create_publisher(Bool, 'human_close', 10)
        self.dist_pub_ = self.create_publisher(Float32, 'human_distance', 10)
        self.angle_pub_ = self.create_publisher(Float32, 'human_angle', 10)
        self.info_pub_ = self.create_publisher(String, 'human_info', 10)
        self.wall_pub_ = self.create_publisher(Bool, 'wall_in_view', 10)

        self.bridge_ = CvBridge()
        self.lock_ = threading.Lock()
        self.latest_color_ = None
        self.latest_depth_ = None
        self.color_seq_ = 0

        self.create_subscription(
            Image, 'depth_camera/image', self.color_cb, 10)
        self.create_subscription(
            Image, 'depth_camera/depth_image', self.depth_cb, 10)

        # --- robot proximity (to suppress robot-as-human false positives) ---
        self.robot_dist_ = float('inf')
        self.robot_angle_ = 0.0
        self.create_subscription(
            Float32, 'nearest_robot_dist', self.rdist_cb, 10)
        self.create_subscription(
            Float32, 'robot_angle', self.rang_cb, 10)

        # --- YOLO model: load now, or wait for the Nav2 readiness gate -------
        # Loading the model costs a torch import, a CUDA context and VRAM per
        # rover.  With 5 rovers that is a multi-second CPU+GPU burst, and it has
        # repeatedly landed inside bring-up or a recovery window and made things
        # worse (2026-09-23: five workers fired at 200 s while a full Nav2/SLAM
        # restart was running, and at 300 s while a surgical restart ran).  A
        # fixed delay only moves the problem, because the gate takes ~60 s in a
        # good run and 400 s in a bad one.
        # YOLO_WAIT_GATE=1 makes it EVENT-DRIVEN: the detector idles until
        # start_experiment.sh publishes /swarm_ready once the gate has passed —
        # which is exactly when detections are first needed.
        self.model_ = None
        self.model_lock_ = threading.Lock()
        if os.environ.get('YOLO_WAIT_GATE', '0') == '1':
            from std_msgs.msg import Bool as _Bool
            from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                                   QoSReliabilityPolicy)
            # ABSOLUTE topic: the detector nodes live in /robot_N namespaces, so
            # a relative name would resolve to /robot_N/swarm_ready and never
            # match the single signal the launcher publishes.
            # TRANSIENT_LOCAL ("latched"): the launcher can publish the gate
            # BEFORE this process exists — the detectors are delayed, and since
            # the sequential bring-up a healthy gate passes in ~100 s while the
            # detectors start at ~200 s.  A volatile one-shot signal was missed
            # by all four detectors on 2026-09-23 11:44, and that whole run
            # recorded human_detected=0 / human_distance=-1 while the bumper log
            # showed 15+ contacts with human_*_bumper.  A latched publisher
            # hands the last value to ANY later subscriber.
            gate_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.gate_topic_ = os.environ.get('YOLO_GATE_TOPIC', '/swarm_ready')
            self.create_subscription(_Bool, self.gate_topic_, self.gate_cb,
                                     gate_qos)
            self.get_logger().info(
                f'detector idle — waiting for the Nav2 gate on '
                f'{self.gate_topic_} (YOLO_WAIT_GATE=1): no model, no CUDA '
                f'context, no inference until navigation is live')
            # SAFETY NET: a gate that never arrives must not silently remove
            # human detection from a whole dataset.  Load anyway after
            # YOLO_GATE_FALLBACK_S seconds (0 disables) and say so loudly.
            self.gate_fallback_s_ = float(
                os.environ.get('YOLO_GATE_FALLBACK_S', '240'))
            if self.gate_fallback_s_ > 0:
                self.create_timer(self.gate_fallback_s_, self.gate_fallback_cb)
        else:
            self.load_model()

        self.worker_ = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_.start()
        self.last_log_ = 0.0
        self.last_state_ = 'NO_HUMAN'
        self.get_logger().info('YoloHumanProcessor started')

    def gate_fallback_cb(self):
        """Load the model even if /swarm_ready never arrived (safety net)."""
        if self.model_ is None:
            self.get_logger().warn(
                f'/swarm_ready not received within {self.gate_fallback_s_:.0f}s '
                f'— loading YOLO anyway so human detection is not silently lost')
            self.load_model()

    def gate_cb(self, msg):
        """Nav2 readiness gate passed -> load YOLO and start detecting."""
        if bool(msg.data) and self.model_ is None:
            self.get_logger().info(
                'Nav2 gate passed — loading YOLO now (detections begin)')
            self.load_model()

    def load_model(self):
        """Load the YOLO model.  Idempotent and safe to call from a callback."""
        with self.model_lock_:
            if self.model_ is not None:
                return
            from ultralytics import YOLO
            import torch
            if self.torch_threads_ > 0:
                torch.set_num_threads(self.torch_threads_)
                # ultralytics calls torch.set_num_threads(NUM_THREADS) every
                # time it selects a CPU device (select_device), which would undo
                # the cap above and hand every rover 8 threads (5 rovers = 40
                # threads on a 16-thread box).  Align its constant as well so
                # the CPU fallback stays bounded; on CUDA that path is never
                # taken.
                try:
                    import ultralytics.utils.torch_utils as _tu
                    _tu.NUM_THREADS = self.torch_threads_
                except Exception:
                    pass
            self.get_logger().info(
                f'Loading YOLO model {self.model_path_} on {self.device_} '
                f'(torch_threads={self.torch_threads_}, half={self.half_}, '
                f'period={self.process_period_:.2f}s) ...')
            self.model_ = YOLO(self.model_path_)
            self.get_logger().info('YOLO model ready')

    def _publish(self, detected, close, dist, angle, state):
        b = Bool(); b.data = detected
        self.det_pub_.publish(b)
        b = Bool(); b.data = close
        self.close_pub_.publish(b)
        f = Float32(); f.data = float(dist)
        self.dist_pub_.publish(f)
        f = Float32(); f.data = float(angle)
        self.angle_pub_.publish(f)
        s = String(); s.data = state
        self.info_pub_.publish(s)
        w = Bool(); w.data = False
        self.wall_pub_.publish(w)

        # throttled human-readable log (same style as image_human_processor)
        now = time.time()
        if detected and (now - self.last_log_) > 0.5:
            side = 'left' if angle > 5.0 else ('right' if angle < -5.0 else 'front')
            self.get_logger().info(
                f'HUMAN at {dist:.2f} m, {angle:.1f} deg ({side})'
                f'{"  [CLOSE]" if close else ""}')
            self.last_log_ = now
            self.last_state_ = 'HUMAN'
        elif not detected and self.last_state_ == 'HUMAN':
            self.get_logger().info('view clear')
            self.last_state_ = 'NO_HUMAN'

    def color_cb(self, msg):
        try:
            img = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.lock_:
            self.latest_color_ = img
            self.color_seq_ += 1

    def rdist_cb(self, msg):
        self.robot_dist_ = float(msg.data)

    def rang_cb(self, msg):
        self.robot_angle_ = float(msg.data)

    def depth_cb(self, msg):
        try:
            d = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        if d.dtype == np.uint16:
            d = d.astype(np.float32) * 0.001  # mm -> m
        else:
            d = d.astype(np.float32)
        with self.lock_:
            self.latest_depth_ = d

    # ------------------------------------------------------------------ #
    def read_depth_at(self, depth, cx, cy, box):
        """Distance at the box centre; fall back to a small cross around it,
        then to the median of the box if the centre is invalid (0/NaN)."""
        h, w = depth.shape[:2]
        candidates = [(cy, cx), (cy, cx + 1), (cy, cx - 1),
                      (cy + 1, cx), (cy - 1, cx)]
        for y, x in candidates:
            if 0 <= y < h and 0 <= x < w:
                v = float(depth[y, x])
                if 0.2 < v < 50.0:
                    return v
        y0, y1 = max(0, box[1]), min(h, box[1] + box[3])
        x0, x1 = max(0, box[0]), min(w, box[0] + box[2])
        region = depth[y0:y1, x0:x1]
        vals = region[(region > 0.2) & (region < 50.0)]
        if vals.size:
            return float(np.median(vals))
        return -1.0

    def worker_loop(self):
        last_det = 0.0
        last_dist = -1.0
        last_angle = 0.0
        last_close = False
        last_seq = -1

        while rclpy.ok():
            with self.lock_:
                color = self.latest_color_
                depth = self.latest_depth_
                seq = self.color_seq_
            if color is None or depth is None or seq == last_seq:
                time.sleep(0.05)
                continue
            last_seq = seq

            # Rate-limit inference (see process_period_sec): drop frames that
            # arrive faster than the configured period.
            if self.process_period_ > 0.0:
                now_inf = time.monotonic()
                if (now_inf - self.last_infer_) < self.process_period_:
                    continue
                self.last_infer_ = now_inf

            img_h, img_w = color.shape[:2]
            fx = (img_w / 2.0) / math.tan(self.hfov_ / 2.0)

            if self.model_ is None:
                # Event-driven detector (YOLO_WAIT_GATE=1): the Nav2 gate has
                # not passed yet, so there is nothing to detect for.  Idle
                # without touching the GPU.
                time.sleep(0.2)
                continue

            results = self.model_.predict(
                color, imgsz=self.input_size_, verbose=False, conf=self.conf_,
                device=self.device_, half=self.half_)
            boxes = results[0].boxes

            best = None  # (area, xyxy)
            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                cls = boxes.cls.cpu().numpy().astype(int)
                for b, c, cl in zip(xyxy, confs, cls):
                    if cl == 0:  # person
                        area = (b[2] - b[0]) * (b[3] - b[1])
                        if best is None or area > best[0]:
                            best = (area, b)

            if best is not None:
                _, (x1, y1, x2, y2) = best
                cx = int((x1 + x2) / 2.0)
                cy = int((y1 + y2) / 2.0)
                dist = self.read_depth_at(
                    depth, cx, cy, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
                angle = -(cx - img_w / 2.0) / fx * 180.0 / math.pi
                close = 0.0 < dist < self.close_dist_
                now = time.time()

                # SUPPRESS robot-as-human false positives: if a real robot is
                # in roughly the same direction, the "person" is the rover's
                # body (YOLO false positive).  Publish NO_HUMAN.
                # Range 6.0 m: robot_proximity tracks the true (world-pose)
                # distance to the nearest robot, and the depth read stays
                # ~1.5 m even for a rover several metres away — so a narrow
                # 2.5 m gate let a facing rover through as a "human".
                robot_suppress = (
                    self.robot_dist_ < 6.0 and
                    abs(angle - self.robot_angle_) < 40.0)
                if robot_suppress:
                    last_det = now  # do NOT hold a suppressed detection
                    self._publish(False, False, -1.0, 0.0, 'ROBOT_NOT_HUMAN')
                else:
                    last_det = now
                    last_dist = dist
                    last_angle = angle
                    last_close = close
                    self._publish(True, close, dist, angle, 'HUMAN')
            else:
                now = time.time()
                if last_det > 0 and (now - last_det) < self.hold_s_:
                    self._publish(True, last_close, last_dist, last_angle, 'HUMAN')
                else:
                    self._publish(False, False, -1.0, 0.0, 'NO_HUMAN')

            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
