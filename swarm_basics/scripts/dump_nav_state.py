#!/usr/bin/env python3
"""One-shot diagnostic: dump global-costmap state for each rover.

Run inside the container while an experiment is live:
    python3 scripts/dump_nav_state.py

Prints per rover: costmap span, unknown/lethal percentages, value histogram,
odom pose (map frame), and cost samples along the straight line from the
stall point (~1.2 m from spawn) to the fixed-goals scan goal (3.0, 0.0) in
each rover's own map frame.
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage

# Robot set + spawn poses come from the shared config (scenario-driven, so
# 1..5 rovers all work); fallback = old 3-rover layout if the package is not
# importable (script run without sourcing install/setup.bash).
try:
    from swarm_basics.robot_config import ROBOT_POSITIONS
    ROBOTS = tuple(ns for ns, *_ in ROBOT_POSITIONS)
    SPAWNS = {ns: (x, y, yaw) for ns, x, y, yaw in ROBOT_POSITIONS}
except Exception:
    ROBOTS = ('robot_0', 'robot_1', 'robot_2')
    SPAWNS = {
        'robot_0': (-4.5, -4.5, 0.7854),
        'robot_1': (4.5, -4.5, 2.3562),
        'robot_2': (-4.5, 4.5, -0.7854),
    }
HUMANS = ('human_1', 'human_2', 'human_1_bumper', 'human_2_bumper')
START = (1.2, 0.0)   # where robots 0/1 stalled (map frame)
GOAL = (3.0, 0.0)    # fixed_goals scan goal #0 (map frame)


def world_to_map(ns, wx, wy):
    x0, y0, yaw0 = SPAWNS[ns]
    dx, dy = wx - x0, wy - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    return dx * c + dy * s, -dx * s + dy * c


class Dump(Node):
    def __init__(self):
        super().__init__('dump_nav_state')
        self.grids = {}
        self.odoms = {}
        self.humans = {}
        for ns in ROBOTS:
            self.create_subscription(
                OccupancyGrid, f'/{ns}/global_costmap/costmap',
                lambda m, ns=ns: self.grids.__setitem__(ns, m), 10)
            self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda m, ns=ns: self.odoms.__setitem__(ns, m), 10)
        self.create_subscription(
            TFMessage, '/world/human_world/dynamic_pose/info',
            self._pose_cb, 10)

    def _pose_cb(self, msg):
        for tf in msg.transforms:
            name = tf.child_frame_id.lstrip('/')
            t = tf.transform.translation
            if name in HUMANS:
                self.humans[name] = (t.x, t.y, t.z)

    def summarize(self, ns, msg):
        info = msg.info
        res = info.resolution
        w, h = info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y
        data = msg.data
        n = len(data)
        hist = {}
        for v in data:
            hist[v] = hist.get(v, 0) + 1
        unknown = hist.get(-1, 0)
        lethal = hist.get(-2, 0) + hist.get(-3, 0)
        print(f'--- {ns} ---')
        print(f'  size {w}x{h} res {res:.3f} origin ({ox:.2f},{oy:.2f}) '
              f'span x[{ox:.1f},{ox + w * res:.1f}] y[{oy:.1f},{oy + h * res:.1f}]')
        print(f'  unknown(-1) {unknown * 100.0 / n:.1f}%  '
              f'lethal/inscribed(-2/-3) {lethal * 100.0 / n:.2f}%')
        top = sorted(hist.items(), key=lambda kv: -kv[1])[:6]
        print(f'  value histogram top: {top}')
        od = self.odoms.get(ns)
        if od:
            p = od.pose.pose.position
            print(f'  odom/map-frame pose: ({p.x:.2f}, {p.y:.2f})')

        def cell_value(x, y):
            cx = int((x - ox) / res)
            cy = int((y - oy) / res)
            if not (0 <= cx < w and 0 <= cy < h):
                return 'OUT'
            return data[cy * w + cx]

        def cell_3x3(x, y):
            cx = int((x - ox) / res)
            cy = int((y - oy) / res)
            vals = []
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    jx, jy = cx + dx, cy + dy
                    if 0 <= jx < w and 0 <= jy < h:
                        vals.append(data[jy * w + jx])
            return f'{min(vals)}..{max(vals)}' if vals else 'OUT'

        print(f'  start {START}: val {cell_value(*START)} 3x3 {cell_3x3(*START)}')
        print(f'  goal  {GOAL}: val {cell_value(*GOAL)} 3x3 {cell_3x3(*GOAL)}')
        od = self.odoms.get(ns)
        if od:
            p = od.pose.pose.position
            print(f'  pose ({p.x:.2f},{p.y:.2f}): val {cell_value(p.x, p.y)} '
                  f'3x3 {cell_3x3(p.x, p.y)}')
        print('  start->goal samples: val | 3x3 range')
        for i in range(13):
            t = i / 12.0
            x = START[0] + t * (GOAL[0] - START[0])
            y = START[1] + t * (GOAL[1] - START[1])
            print(f'    t={t:.2f} d={t * 1.8:.2f}m val={cell_value(x, y)} '
                  f'3x3={cell_3x3(x, y)}')
        for hn, hw in sorted(self.humans.items()):
            hx, hy = world_to_map(ns, hw[0], hw[1])
            print(f'  {hn} world({hw[0]:.2f},{hw[1]:.2f}) '
                  f'-> map({hx:.2f},{hy:.2f}) val {cell_value(hx, hy)} '
                  f'3x3 {cell_3x3(hx, hy)}')


def main():
    rclpy.init()
    node = Dump()
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(node.grids) == len(ROBOTS) and len(node.odoms) == len(ROBOTS) \
                and len(node.humans) >= 2 and time.time() - t0 > 6.0:
            break
    print(f'=== nav state dump (waited {time.time() - t0:.1f}s, '
          f'{len(node.grids)}/3 costmaps, {len(node.odoms)}/3 odoms, '
          f'{len(node.humans)} human poses) ===')
    print('human world poses: ' + ', '.join(
        f'{k}=({v[0]:.2f},{v[1]:.2f})' for k, v in sorted(node.humans.items())))
    for ns in ROBOTS:
        msg = node.grids.get(ns)
        if msg is None:
            print(f'--- {ns} --- NO COSTMAP RECEIVED')
            continue
        node.summarize(ns, msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
