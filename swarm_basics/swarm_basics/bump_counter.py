#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import UInt32, String
from ros_gz_interfaces.msg import Contacts

COOLDOWN_SEC = 0.5
PRUNE_HZ = 10.0
CONTACT_TOPIC = 'contact'

def entity_name(ent) -> str:
    try:
        return ent.name
    except AttributeError:
        return str(ent)

class BumpCounter(Node):
    """Detects bumps and publishes bump_count + last_bump_with.
       No file I/O — coverage_plotter handles CSV logging."""
    def __init__(self):
        super().__init__('bump_counter')
        self.ns = self.get_namespace().strip('/') or 'root'
        self.cooldown = Duration(seconds=COOLDOWN_SEC)
        self.bump_count = 0
        self.active = {}
        self.sub = self.create_subscription(Contacts, CONTACT_TOPIC, self.on_contacts, 10)
        self.pub_count = self.create_publisher(UInt32, 'bump_count', 10)
        self.pub_last  = self.create_publisher(String, 'last_bump_with', 10)
        self.create_timer(1.0/PRUNE_HZ, self.prune)
        self.get_logger().info(f"[{self.ns}] BumpCounter active")

    def on_contacts(self, msg: Contacts):
        now = self.get_clock().now()
        for c in msg.contacts:
            col1 = entity_name(c.collision1)
            col2 = entity_name(c.collision2)
            is1_me = self.ns in col1
            is2_me = self.ns in col2
            if is1_me == is2_me:
                continue
            self_col = col1 if is1_me else col2
            other_col = col2 if is1_me else col1
            if other_col not in self.active:
                self.bump_count += 1
                self.active[other_col] = now
                self.pub_count.publish(UInt32(data=self.bump_count))
                self.pub_last.publish(String(data=other_col))
                # coverage_plotter receives bump_count and logs to CSV
                self.get_logger().info(f"[{self.ns}] bump #{self.bump_count} with: {other_col}")

    def prune(self):
        now = self.get_clock().now()
        expired = [k for k, v in self.active.items() if now - v > self.cooldown]
        for k in expired:
            del self.active[k]

def main(args=None):
    rclpy.init(args=args)
    node = BumpCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
