#!/usr/bin/env python3
"""clock_throttle — republish Gazebo's physics-rate clock at a sane rate.

Gazebo publishes one `rosgraph_msgs/Clock` message per PHYSICS STEP (the world's
max_step_size, 1 ms here), and `ros_gz_bridge` forwards every single one to ROS.

MEASURED 2026-09-23 (4 rovers, RTF 0.38): **383 Hz on /clock**.  Every node
running with `use_sim_time:=true` subscribes to it, so with ~15 sim-time nodes
per rover that is tens of thousands of callbacks per second whose only job is to
advance the clock.  The executors then starve, and the symptoms look like
unrelated bugs:

  * Nav2 costmaps frozen in 'Activating' with their clock stuck at 0.7 s
    (the sim time they saw at startup) -> "Timed out waiting for transform ...
    Lookup would require extrapolation into the past. Requested time 0.700000
    but the earliest data is at time 79.750000"
  * slam_toolbox overflowing its message-filter queue -> "discarding message
    because the queue is full" -> no map -> the readiness gate can never pass
  * lifecycle "failed to send response to .../get_state (timeout)" aborts

None of that is a logic bug: it is timekeeping traffic crowding out real work.

Sim time does not need 383 Hz.  Control runs at 20 Hz, the Nav2 BT tick at
100 Hz, and mapping reacts in the hundreds of milliseconds.  This node keeps the
NEWEST sample and republishes it at `rate_hz` (default 50), cutting the feed ~8x
with zero fidelity cost: time still advances monotonically, consumers just see
it in 20 ms steps instead of 2.6 ms ones.

Wiring (launch/leo_gz.launch.py): the clock bridge's ROS side is remapped to
/clock_raw so this node owns /clock.

    /clock_raw (bridge, ~383 Hz)  -->  clock_throttle  -->  /clock (50 Hz)

The publisher is RELIABLE on purpose: a reliable publisher satisfies BOTH
reliable and best-effort subscribers, whereas best-effort would silently starve
any consumer that asks for reliability (rclpy's TimeSource does).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock


class ClockThrottle(Node):
    def __init__(self):
        super().__init__('clock_throttle')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('input_topic', '/clock_raw')
        self.declare_parameter('output_topic', '/clock')
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Depth-1 KEEP_LAST everywhere: a queue of clock messages is worse than
        # useless, we only ever want the newest value.
        sub_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Clock, out_topic, pub_qos)
        self.create_subscription(Clock, in_topic, self._on_clock, sub_qos)
        self.latest = None
        self.rx = 0
        self.tx = 0
        self.create_timer(1.0 / rate, self._publish_latest)
        self.get_logger().info(
            f'relaying {in_topic} -> {out_topic} at {rate:.0f} Hz '
            f'(Gazebo emits one Clock per physics step; sim time does not need '
            f'that resolution)')

    def _on_clock(self, msg: Clock) -> None:
        self.latest = msg
        self.rx += 1

    def _publish_latest(self) -> None:
        if self.latest is None:
            return
        self.pub.publish(self.latest)
        self.tx += 1


def main(args=None):
    rclpy.init(args=args)
    node = ClockThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
