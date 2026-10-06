#!/usr/bin/env python3
"""wait_nav2_ready.py — robust Nav2 + SLAM readiness probe for start_experiment.sh.

HARD health check (2026-09-17): a robot counts as ready only when BOTH
  1) its '<ns>/bt_navigator' lifecycle node reports 'active' (the lifecycle
     manager activates planner/controller/behavior/bt_navigator in order, so
     this implies the whole Nav2 chain is up), and
  2) at least one message has arrived on '<ns>/map' — proof that the robot's
     SLAM is actually RUNNING, not merely spawned.
The recurring startup flake (2026-09-14..17) wedged a single slam_toolbox
process at init (futex wait, zero log output); the map check exposes exactly
that state, and start_experiment.sh restarts the Nav2/SLAM stack when this
probe fails.

EARLY EXIT (2026-09-17) — fail faster than the full timeout:
  * --nav2-log PATH : watch NEW nav2.log output for 'Aborting bringup'
    (lifecycle manager gave up — that robot can never become ready now).
  * --stall SECS    : default 90; if at least one robot is fully ready and
    some other robot shows NO progress (no nav active, no map) for that
    long, exit early naming the straggler(s).  0 disables the check.
  * --result-file PATH : write 'OK' or 'FAILED <ns> <ns> ...' (robots to
    restart) — start_experiment.sh drives its surgical recovery from it.

This probe is ONE rclpy process: it discovers each robot's bt_navigator
lifecycle service once and polls it until active, and subscribes to
'<ns>/map' until the first grid arrives.  One line is printed per robot the
moment both conditions are met, plus a final READY / TIMEOUT / STALL /
ABORTED line listing exactly what is missing per robot.
Exit code: 0 = all conditions met, 1 = not ready (timeout/stall/aborted),
2 = usage error.
"""

import re
import sys
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# 'Aborting bringup' => a lifecycle manager gave up; the robot can never
# become ready in this attempt (the manager does not retry by itself).
ABORT_RE = re.compile(
    r'\[(robot_[A-Za-z0-9_]+)\.lifecycle_manager\][^\n]*Aborting bringup')

# How long to wait for one get_state reply before assuming it was LOST and
# asking again.  A reply can genuinely be dropped under load (nav2.log shows
# 'failed to send response to /robot_3/bt_navigator/get_state (timeout): client
# will not receive response'), and without this retry the probe waits on that
# never-completing future for the rest of the attempt — so a robot that HAS
# recovered is still reported as failed and the gate can never pass.
REQUEST_TTL = 10.0


def parse_args(argv):
    """[--stall SECS] [--nav2-log PATH] [--result-file PATH] <timeout> <ns>..."""
    timeout, stall, nav2_log, result_file = None, 90.0, None, None
    robots = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '--stall':
            stall = float(argv[i + 1]); i += 2
        elif arg == '--nav2-log':
            nav2_log = argv[i + 1]; i += 2
        elif arg == '--result-file':
            result_file = argv[i + 1]; i += 2
        elif timeout is None:
            timeout = float(arg); i += 1
        else:
            robots.append(arg); i += 1
    return timeout, stall, nav2_log, result_file, robots


def scan_abort(handle):
    """Robot name(s) whose bringup aborted in NEW nav2.log output, else None."""
    if handle is None:
        return None
    chunk = handle.read()
    if not chunk or 'Aborting bringup' not in chunk:
        return None
    found = ABORT_RE.findall(chunk)
    return ','.join(dict.fromkeys(found)) if found else 'unknown'


def write_result(path, ok, failed, noscan=()):
    """Machine-readable gate outcome for start_experiment.sh recovery.

    Line 1  'OK' | 'FAILED <ns> ...'   robots to restart (contract unchanged:
                                        failed_robots() reads exactly this).
    Line 2  'NOSCAN <ns> ...'          optional; those robots never received a
                                        LiDAR scan at all.  That is a SIM-side
                                        fault (lidar_republish / Gazebo), which
                                        a Nav2 restart can never repair — so the
                                        shell re-starts the republisher instead.
    """
    if not path:
        return
    try:
        with open(path, 'w') as fh:
            fh.write('OK\n' if ok else 'FAILED ' + ' '.join(failed) + '\n')
            if not ok and noscan:
                fh.write('NOSCAN ' + ' '.join(noscan) + '\n')
    except OSError:
        pass


def main():
    timeout, stall, nav2_log, result_file, robots = parse_args(sys.argv[1:])
    if timeout is None or not robots:
        print('usage: wait_nav2_ready.py [--stall SECS] [--nav2-log PATH] '
              '[--result-file PATH] <timeout_s> <ns> [<ns> ...]', file=sys.stderr)
        return 2

    rclpy.init()
    node = Node('nav2_ready_probe')

    clients = {
        ns: node.create_client(GetState, f'/{ns}/bt_navigator/get_state')
        for ns in robots
    }
    nav_ok = {ns: False for ns in robots}
    map_ok = {ns: False for ns in robots}
    scan_ok = {ns: False for ns in robots}
    start = time.time()
    last_progress = {ns: start for ns in robots}   # last nav-active/first-map

    def on_map(_msg, ns=None):
        if not map_ok[ns]:
            map_ok[ns] = True
            last_progress[ns] = time.time()

    # Is the SIM actually feeding this robot?  Without a scan there can be no
    # map, and the two causes need opposite repairs (republisher/Gazebo restart
    # vs Nav2 restart), so they must not look identical in the log.
    def on_scan(_msg, ns=None):
        if not scan_ok[ns]:
            scan_ok[ns] = True
            print(f'{ns}: scan flowing', flush=True)

    for ns in robots:
        node.create_subscription(
            OccupancyGrid, f'/{ns}/map', lambda msg, ns=ns: on_map(msg, ns), 10)
        node.create_subscription(
            LaserScan, f'/{ns}/lidar/scan_clean',
            lambda msg, ns=ns: on_scan(msg, ns), 10)

    # Watch only NEW nav2.log output: the probe can run a second time after a
    # restart appended to the same file — old attempts must not trigger again.
    nav2_log_fh = None
    try:
        if nav2_log:
            nav2_log_fh = open(nav2_log, 'r', errors='replace')
            nav2_log_fh.seek(0, 2)
    except OSError:
        nav2_log_fh = None

    pending_futures = {}         # ns -> in-flight GetState future
    pending_sent = {}            # ns -> when that request went out (REQUEST_TTL)
    logged_done = set()          # robots whose success line was printed
    stalled = []                 # early exit: robots with no progress
    abort_robot = None           # early exit: lifecycle bringup aborted
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()

            for ns in robots:
                if nav_ok[ns] or ns in pending_futures:
                    continue
                cli = clients[ns]
                if cli.service_is_ready():
                    pending_futures[ns] = cli.call_async(GetState.Request())
                    pending_sent[ns] = now

            for ns, fut in list(pending_futures.items()):
                if not fut.done():
                    # A request whose reply never arrives must not wedge the probe:
                    # the future stays pending forever, no further request is ever
                    # sent, and a robot that HAS recovered is still reported as
                    # failed.  Observed 2026-09-23: the probe held a pending
                    # get_state for robot_3 ACROSS the surgical restart that healed
                    # it (nav2.log: 'failed to send response to
                    # /robot_3/bt_navigator/get_state (timeout): client will not
                    # receive response') and the gate failed a healthy robot —
                    # which is exactly why the laggard looked impossible to clear.
                    # Give up after REQUEST_TTL and ask again.
                    if now - pending_sent.get(ns, now) > REQUEST_TTL:
                        del pending_futures[ns]
                        pending_sent.pop(ns, None)
                        print(f'{ns}: get_state response lost - retrying', flush=True)
                    continue
                del pending_futures[ns]
                pending_sent.pop(ns, None)
                res = fut.result()
                if res is not None and res.current_state.label == 'active':
                    nav_ok[ns] = True
                    last_progress[ns] = now
                    print(f'{ns}: bt_navigator active', flush=True)

            for ns in robots:
                if ns not in logged_done and nav_ok[ns] and map_ok[ns]:
                    logged_done.add(ns)
                    print(f'{ns}: ACTIVE + map OK', flush=True)

            if all(nav_ok[ns] and map_ok[ns] for ns in robots):
                break

            # EARLY EXIT 1 — a lifecycle manager gave up ("Aborting bringup"
            # in nav2.log): that robot stays inactive for this whole attempt.
            abort_robot = scan_abort(nav2_log_fh)
            if abort_robot:
                break

            # EARLY EXIT 2 — stall: at least one robot is fully ready, but
            # some robot shows NO progress (no nav, no map) for --stall secs.
            if stall > 0 and logged_done:
                stalled = [ns for ns in robots
                           if ns not in logged_done
                           and now - last_progress[ns] >= stall]
                if stalled:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        waiting = []
        for ns in robots:
            missing = []
            if not nav_ok[ns]:
                missing.append('nav2')
            if not map_ok[ns]:
                missing.append('map')
            if not scan_ok[ns]:
                missing.append('no_scan')
            if missing:
                waiting.append(f'{ns}({",".join(missing)})')
        ok = not waiting
        failed = []
        if ok:
            print('READY', flush=True)
        else:
            if abort_robot and abort_robot != 'unknown':
                failed = [r for r in abort_robot.split(',') if r]
            elif stalled:
                failed = list(stalled)
            else:
                failed = [ns for ns in robots
                          if not (nav_ok[ns] and map_ok[ns])]
            if abort_robot:
                print(f'ABORTED — lifecycle bringup aborted for: {abort_robot} '
                      f'(still waiting: {waiting})', flush=True)
            elif stalled:
                print(f'STALL — no progress from: {stalled} for {stall:.0f}s '
                      f'({len(logged_done)} robot(s) ready)', flush=True)
            else:
                print(f'TIMEOUT — still waiting for: {waiting}', flush=True)
        write_result(result_file, ok, failed,
                     noscan=[ns for ns in robots if not scan_ok[ns]])
        if nav2_log_fh is not None:
            nav2_log_fh.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
