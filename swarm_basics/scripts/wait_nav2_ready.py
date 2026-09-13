#!/usr/bin/env python3
"""wait_nav2_ready.py — robust Nav2 readiness probe for start_experiment.sh.

Replaces polling `ros2 lifecycle get` (a fresh CLI process per poll): under
boot load those one-shot CLIs often miss discovery within their timeout and
the gate then reports "not active" for stacks that ARE active (observed
2026-09-13: both robots' lifecycle managers reported fully active 14 s / 23 s
after launch, the gate was still waiting 40 s later — the run got
interrupted before any goal was ever sent).

This probe is ONE rclpy process: it discovers each robot's bt_navigator
lifecycle service once, then queries '<ns>/bt_navigator/get_state' until
every robot reports 'active' or the timeout expires.  One line is printed per
robot the moment it turns active, plus a final READY / TIMEOUT line.
Exit code: 0 = all active, 1 = timeout, 2 = usage error.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState


def main():
    if len(sys.argv) < 3:
        print('usage: wait_nav2_ready.py <timeout_s> <ns> [<ns> ...]',
              file=sys.stderr)
        return 2
    timeout = float(sys.argv[1])
    robots = sys.argv[2:]

    rclpy.init()
    node = Node('nav2_ready_probe')
    clients = {
        ns: node.create_client(GetState, f'/{ns}/bt_navigator/get_state')
        for ns in robots
    }
    pending = set(robots)
    deadline = time.time() + timeout
    try:
        while pending and time.time() < deadline:
            for ns in sorted(pending):
                cli = clients[ns]
                if not cli.service_is_ready():
                    cli.wait_for_service(timeout_sec=1.0)
                    continue
                fut = cli.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                if (fut.done() and fut.result() is not None
                        and fut.result().current_state.label == 'active'):
                    pending.discard(ns)
                    print(f'{ns}: bt_navigator active', flush=True)
            if pending:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ok = not pending
        if ok:
            print('READY', flush=True)
        else:
            print(f'TIMEOUT — still waiting for: {sorted(pending)}', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
