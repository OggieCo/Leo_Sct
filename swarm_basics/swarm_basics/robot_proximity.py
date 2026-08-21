#!/usr/bin/env python3
"""robot_proximity — cone-aware robot-robot proximity perception.

Subscribes to the global pose topic (/world/<WORLD_NAME>/dynamic_pose/info,
a TFMessage with all robot model frames broadcast by Gazebo) and publishes,
per robot namespace:

  robot_close          (std_msgs/Bool)    any other robot < close_distance
  robot_angle          (std_msgs/Float32) bearing of nearest robot, deg (+left)
  nearest_robot_dist   (std_msgs/Float32) distance to the nearest other robot
  nearest_robot_id     (std_msgs/String)  name of the nearest other robot

Cone-aware: among OTHER robots it prefers the closest one inside
±cone_half_deg of the forward axis, so a robot dead ahead triggers a yield
even if a side robot is equidistant.  Feeds the IsRobotClose BT node and the
social_event_logger (ROBOT_NEAR / ROBOT_GONE events).
"""

import math

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, Float32, String

from swarm_basics.robot_config import WORLD_NAME, ROBOT_POSITIONS


def quat_to_yaw(q):
    """Yaw (rad) from a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotProximity(Node):
    def __init__(self):
        super().__init__('robot_proximity')
        self.ns = self.get_namespace().strip('/') or 'root'

        self.declare_parameter('close_distance', 2.4)  # 2.0 -> 2.4 (20% further)
        self.declare_parameter('cone_half_deg', 30.0)

        self.other_robots = [n for n, _, _, _ in ROBOT_POSITIONS if n != self.ns]

        self._pub_close = self.create_publisher(Bool, 'robot_close', 10)
        self._pub_angle = self.create_publisher(Float32, 'robot_angle', 10)
        self._pub_dist = self.create_publisher(Float32, 'nearest_robot_dist', 10)
        self._pub_dca = self.create_publisher(Float32, 'robot_dca', 10)
        self._pub_faster = self.create_publisher(Bool, 'robot_faster', 10)
        self._pub_id = self.create_publisher(String, 'nearest_robot_id', 10)

        self._hist = {}   # name -> list[(t_ns, x, y)] for velocity estimation

        self._sub = self.create_subscription(
            TFMessage, f'/world/{WORLD_NAME}/dynamic_pose/info',
            self.pose_cb, 10)

        self.get_logger().info(
            f'RobotProximity [{self.ns}]: watching {self.other_robots}, '
            f'close < {self.get_parameter("close_distance").value:.2f} m, '
            f'cone +/- {self.get_parameter("cone_half_deg").value:.0f} deg')

    def pose_cb(self, msg):
        poses = {}   # name -> (x, y, yaw)
        for t in msg.transforms:
            name = t.child_frame_id
            if name in self.other_robots or name == self.ns:
                p = t.transform.translation
                poses[name] = (p.x, p.y, quat_to_yaw(t.transform.rotation))

        if self.ns not in poses or not self.other_robots:
            return

        my_x, my_y, my_yaw = poses[self.ns]

        # relative bearing + distance to every other robot
        candidates = []
        for name in self.other_robots:
            if name not in poses:
                continue
            ox, oy, _ = poses[name]
            dx, dy = ox - my_x, oy - my_y
            dist = math.hypot(dx, dy)
            rel = math.degrees(math.atan2(dy, dx)) - math.degrees(my_yaw)
            while rel > 180.0:
                rel -= 360.0
            while rel < -180.0:
                rel += 360.0
            candidates.append((name, dist, rel))

        if not candidates:
            return

        cone = self.get_parameter('cone_half_deg').value
        # Prefer the closest robot inside the cone; else the nearest overall.
        in_cone = [c for c in candidates if abs(c[2]) <= cone]
        pool = in_cone if in_cone else candidates
        name, dist, rel = min(pool, key=lambda c: c[1])

        close = dist <= self.get_parameter('close_distance').value

        # --- velocity estimation from pose history (~0.6 s window) ---------
        now = self.get_clock().now().nanoseconds
        vel = {}
        for n, (x, y, _) in poses.items():
            h = self._hist.setdefault(n, [])
            h.append((now, x, y))
            while len(h) > 2 and now - h[0][0] > 0.6e9:
                h.pop(0)
            if len(h) >= 2:
                dt = (h[-1][0] - h[0][0]) / 1e9
                if dt > 1e-3:
                    vel[n] = ((h[-1][1] - h[0][1]) / dt,
                              (h[-1][2] - h[0][2]) / dt)

        # --- predicted closest approach (DCA) + speed comparison -----------
        dca = dist
        faster = False
        if self.ns in vel and name in vel:
            mx, my = vel[self.ns]
            ox_, oy_ = vel[name]
            vrx, vry = ox_ - mx, oy_ - my
            v2 = vrx * vrx + vry * vry
            rx, ry = poses[name][0] - my_x, poses[name][1] - my_y
            if v2 > 1e-6 and rx * vrx + ry * vry < 0.0:   # closing
                tca = -(rx * vrx + ry * vry) / v2
                dca = math.hypot(rx + vrx * tca, ry + vry * tca)
            faster = math.hypot(ox_, oy_) > math.hypot(mx, my) + 0.1

        bc = Bool(); bc.data = close
        ba = Float32(); ba.data = float(rel)
        bd = Float32(); bd.data = float(dist)
        bdca = Float32(); bdca.data = float(dca)
        bf = Bool(); bf.data = faster
        bi = String(); bi.data = name
        self._pub_close.publish(bc)
        self._pub_angle.publish(ba)
        self._pub_dist.publish(bd)
        self._pub_dca.publish(bdca)
        self._pub_faster.publish(bf)
        self._pub_id.publish(bi)


def main(args=None):
    rclpy.init(args=args)
    node = RobotProximity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
