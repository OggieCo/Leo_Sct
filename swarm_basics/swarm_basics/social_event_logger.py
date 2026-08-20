#!/usr/bin/env python3
"""social_event_logger — writes social/behavior CSVs for evaluation.

Writes into the SAME run folder as coverage_plotter
(/root/ros2_ws/src/results/coverage_logs/run_<timestamp>/):

  social_state.csv    periodic snapshot per robot (1 Hz):
                      timestamp,elapsed_sec,robot_id,x,y,yaw,
                      human_detected,human_close,human_distance,human_angle,
                      robot_close,robot_angle,nearest_robot_dist,
                      obstacle_min,cmd_lin,cmd_ang,backwards

  social_events.csv   edge-triggered events:
                      timestamp,elapsed_sec,robot_id,event_type,details
                      event_type: HUMAN_SEEN, HUMAN_GONE, OBSTACLE_NEAR,
                      ROBOT_NEAR, ROBOT_GONE, BT (details = BT event text),
                      BUMP, BACKWARD_START, BACKWARD_END

  social_summary.csv  per-robot event counts at shutdown:
                      robot_id,event_type,count

Subscribes to /{ns}/human_* (YOLO), /{ns}/robot_* (robot_proximity),
/{ns}/lidar/scan, /{ns}/cmd_vel, /{ns}/bt_social_event (BT nodes), the
world pose topic and the per-robot contact sensor.
"""

import math
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from ros_gz_interfaces.msg import Contacts

from swarm_basics.robot_config import WORLD_NAME

BASE_DIR = '/root/ros2_ws/src/results/coverage_logs'
STATE_HEADER = ('timestamp,elapsed_sec,robot_id,x,y,yaw,'
                'human_detected,human_close,human_distance,human_angle,'
                'robot_close,robot_angle,nearest_robot_dist,'
                'obstacle_min,cmd_lin,cmd_ang,actual_speed_kph,backwards,speed_scale')
EVENT_HEADER = 'timestamp,elapsed_sec,robot_id,event_type,details'
OBSTACLE_NEAR_THRESHOLD = 0.5
BACKWARD_THRESHOLD = -0.01


class SocialEventLogger(Node):
    def __init__(self):
        super().__init__('social_event_logger')
        self.start_time = time.time()

        from swarm_basics.run_utils import get_run_dir
        self.csv_dir = str(get_run_dir())

        self.state_path = f'{self.csv_dir}/social_state.csv'
        self.event_path = f'{self.csv_dir}/social_events.csv'
        self.summary_path = f'{self.csv_dir}/social_summary.csv'

        with open(self.state_path, 'w') as f:
            f.write(STATE_HEADER + '\n')
        with open(self.event_path, 'w') as f:
            f.write(EVENT_HEADER + '\n')
        with open(self.summary_path, 'w') as f:
            f.write('robot_id,event_type,count\n')

        self.robots = [f'robot_{i}' for i in range(10)]
        # per-robot stored state
        self.state = {}
        for ns in self.robots:
            self.state[ns] = {
                'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'has_pose': False,
                'human_detected': False, 'human_close': False,
                'human_distance': -1.0, 'human_angle': 0.0,
                'robot_close': False, 'robot_angle': 0.0,
                'nearest_robot_dist': float('inf'),
                'obstacle_min': float('inf'),
                'cmd_lin': 0.0, 'cmd_ang': 0.0, 'actual_speed_kph': 0.0,
                'backwards': False,
                'speed_scale': 1.0,
                'prev_human_detected': False, 'prev_robot_close': False,
                'prev_obstacle_min': float('inf'), 'prev_backwards': False,
                'bump_count': 0,
            }

        # world pose
        self.create_subscription(
            TFMessage, f'/world/{WORLD_NAME}/dynamic_pose/info',
            self.pose_cb, 10)

        # per-robot topics
        for ns in self.robots:
            self.create_subscription(Bool, f'/{ns}/human_detected',
                                     lambda m, n=ns: self.set(n, 'human_detected', m.data), 10)
            self.create_subscription(Bool, f'/{ns}/human_close',
                                     lambda m, n=ns: self.set(n, 'human_close', m.data), 10)
            self.create_subscription(Float32, f'/{ns}/human_distance',
                                     lambda m, n=ns: self.set(n, 'human_distance', float(m.data)), 10)
            self.create_subscription(Float32, f'/{ns}/human_angle',
                                     lambda m, n=ns: self.set(n, 'human_angle', float(m.data)), 10)
            self.create_subscription(Bool, f'/{ns}/robot_close',
                                     lambda m, n=ns: self.set(n, 'robot_close', m.data), 10)
            self.create_subscription(Float32, f'/{ns}/robot_angle',
                                     lambda m, n=ns: self.set(n, 'robot_angle', float(m.data)), 10)
            self.create_subscription(Float32, f'/{ns}/nearest_robot_dist',
                                     lambda m, n=ns: self.set(n, 'nearest_robot_dist', float(m.data)), 10)
            self.create_subscription(LaserScan, f'/{ns}/lidar/scan',
                                     lambda m, n=ns: self.scan_cb(n, m), 10)
            self.create_subscription(Twist, f'/{ns}/cmd_vel',
                                     lambda m, n=ns: self.cmd_cb(n, m), 10)
            # Actual ground speed from odometry (m/s -> km/h)
            self.create_subscription(Odometry, f'/{ns}/odom',
                                     lambda m, n=ns: self.set(
                                         n, 'actual_speed_kph',
                                         float(m.twist.twist.linear.x) * 3.6), 10)
            self.create_subscription(Float32, f'/{ns}/social_speed_scale',
                                     lambda m, n=ns: self.set(n, 'speed_scale', float(m.data)), 10)
            self.create_subscription(String, f'/{ns}/bt_social_event',
                                     lambda m, n=ns: self.event(n, 'BT', m.data), 10)
            self.create_subscription(
                Contacts,
                f'/world/{WORLD_NAME}/model/{ns}/link/{ns}/base_footprint/'
                f'sensor/contact_sensor/contact',
                lambda m, n=ns: self.contact_cb(n, m), 10)

        # 1 Hz state snapshot
        self.create_timer(1.0, self.state_tick)

        self.get_logger().info(
            f'SocialEventLogger: logging to {self.csv_dir} (robots={self.robots})')

    # ---- helpers ---------------------------------------------------------
    def set(self, ns, key, value):
        if ns in self.state:
            self.state[ns][key] = value

    def now(self):
        return time.time()

    def elapsed(self):
        return self.now() - self.start_time

    # ---- callbacks -------------------------------------------------------
    def pose_cb(self, msg):
        for t in msg.transforms:
            name = t.child_frame_id
            if name in self.state:
                p = t.transform.translation
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.state[name]['x'] = p.x
                self.state[name]['y'] = p.y
                self.state[name]['yaw'] = yaw
                self.state[name]['has_pose'] = True

    def scan_cb(self, ns, msg):
        finite = [r for r in msg.ranges if math.isfinite(r)]
        mn = min(finite) if finite else float('inf')
        s = self.state[ns]
        prev = s['obstacle_min']
        s['obstacle_min'] = mn
        if prev >= OBSTACLE_NEAR_THRESHOLD and mn < OBSTACLE_NEAR_THRESHOLD:
            self.event(ns, 'OBSTACLE_NEAR', f'min={mn:.2f}')
        if mn >= OBSTACLE_NEAR_THRESHOLD:
            s['obstacle_min'] = float('inf')

    def cmd_cb(self, ns, msg):
        s = self.state[ns]
        s['cmd_lin'] = msg.linear.x
        s['cmd_ang'] = msg.angular.z
        back = msg.linear.x < BACKWARD_THRESHOLD
        if back and not s['backwards']:
            self.event(ns, 'BACKWARD_START', '')
        elif not back and s['backwards']:
            self.event(ns, 'BACKWARD_END', '')
        s['backwards'] = back

    def contact_cb(self, ns, msg):
        s = self.state[ns]
        s['bump_count'] += 1
        self.event(ns, 'BUMP', f'count={s["bump_count"]}')

    # ---- event + state writes -------------------------------------------
    def event(self, ns, etype, details):
        line = f'{self.now():.3f},{self.elapsed():.3f},{ns},{etype},{details}'
        with open(self.event_path, 'a') as f:
            f.write(line + '\n')

    def state_tick(self):
        with open(self.state_path, 'a') as f:
            for ns, s in self.state.items():
                if not s['has_pose']:
                    continue
                # edge events
                if s['human_detected'] and not s['prev_human_detected']:
                    self.event(ns, 'HUMAN_SEEN',
                               f'dist={s["human_distance"]:.2f} angle={s["human_angle"]:.1f}')
                elif not s['human_detected'] and s['prev_human_detected']:
                    self.event(ns, 'HUMAN_GONE', '')
                if s['robot_close'] and not s['prev_robot_close']:
                    self.event(ns, 'ROBOT_NEAR',
                               f'dist={s["nearest_robot_dist"]:.2f} angle={s["robot_angle"]:.1f}')
                elif not s['robot_close'] and s['prev_robot_close']:
                    self.event(ns, 'ROBOT_GONE', '')
                s['prev_human_detected'] = s['human_detected']
                s['prev_robot_close'] = s['robot_close']

                def fmt_dist(v):
                    return 'inf' if math.isinf(v) else f'{v:.2f}'

                row = (f'{self.now():.3f},{self.elapsed():.3f},{ns},'
                       f'{s["x"]:.3f},{s["y"]:.3f},{s["yaw"]:.3f},'
                       f'{1 if s["human_detected"] else 0},'
                       f'{1 if s["human_close"] else 0},'
                       f'{s["human_distance"]:.2f},{s["human_angle"]:.1f},'
                       f'{1 if s["robot_close"] else 0},'
                       f'{s["robot_angle"]:.1f},'
                       f'{fmt_dist(s["nearest_robot_dist"])},'
                       f'{fmt_dist(s["obstacle_min"])},'
                       f'{s["cmd_lin"]:.3f},{s["cmd_ang"]:.3f},'
                       f'{s["actual_speed_kph"]:.2f},'
                       f'{1 if s["backwards"] else 0},'
                       f'{s["speed_scale"]:.3f}\n')
                f.write(row)

    # ---- shutdown summary -----------------------------------------------
    def on_shutdown(self):
        counts = {}
        with open(self.event_path) as f:
            next(f)
            for line in f:
                parts = line.rstrip('\n').split(',')
                if len(parts) >= 4:
                    ns, etype = parts[2], parts[3]
                    counts[(ns, etype)] = counts.get((ns, etype), 0) + 1
        with open(self.summary_path, 'a') as f:
            for (ns, etype), c in sorted(counts.items()):
                f.write(f'{ns},{etype},{c}\n')
        self.get_logger().info('SocialEventLogger: summary written')


def main(args=None):
    rclpy.init(args=args)
    node = SocialEventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
