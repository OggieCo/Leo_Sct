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

        self.close_dist_ = self.get_parameter('close_distance').value
        self.conf_ = self.get_parameter('confidence').value
        self.model_path_ = self.get_parameter('model_path').value
        self.input_size_ = self.get_parameter('input_size').value
        self.hfov_ = self.get_parameter('hfov').value
        self.hold_s_ = self.get_parameter('detection_hold_s').value

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

        # --- load YOLO (downloads yolov8n.pt on first use) ---
        from ultralytics import YOLO
        self.get_logger().info(f'Loading YOLO model {self.model_path_} ...')
        self.model_ = YOLO(self.model_path_)
        self.get_logger().info('YOLO model ready')

        self.worker_ = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_.start()
        self.last_log_ = 0.0
        self.last_state_ = 'NO_HUMAN'
        self.get_logger().info('YoloHumanProcessor started')

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

            img_h, img_w = color.shape[:2]
            fx = (img_w / 2.0) / math.tan(self.hfov_ / 2.0)

            results = self.model_.predict(
                color, imgsz=self.input_size_, verbose=False, conf=self.conf_)
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
