import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, UInt32
from ros_gz_interfaces.msg import Contacts
from swarm_basics.robot_config import WORLD_NAME
import matplotlib
# Headless by default: the live matplotlib GUI window can't be minimized when
# running inside the container.  Render offscreen + save PNG instead.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from matplotlib.patches import Rectangle
from collections import defaultdict
import csv
import time
import math
import signal
from pathlib import Path


class CoveragePlotter(Node):
    def __init__(self):
        super().__init__('coverage_plotter')

        # dynamic_plots=false: skip the periodic redraw/PNG/CSV-refresh timer
        # entirely.  Trajectories/events CSVs are still written continuously;
        # the coverage map + summary/coverage CSVs are rendered once, in
        # save_final_plot() at shutdown.
        self.declare_parameter('dynamic_plots', True)
        self.dynamic_plots = bool(self.get_parameter('dynamic_plots').value)

        # log_raw_contacts=true: ALSO write every raw contact-sensor message to
        # contacts_raw.csv (very chatty: ~90 rows/s while touching).  Default
        # false: events.csv keeps one CONTACT_BEGIN/CONTACT_END row pair per
        # contact episode, so duration + sample counts are preserved.
        self.declare_parameter('log_raw_contacts', False)
        self.log_raw_contacts = bool(
            self.get_parameter('log_raw_contacts').value)

        # === CONFIGURATION ===
        self.robot_namespaces = [f"robot_{i}" for i in range(10)]
        #self.save_path = "/home/ecem/ros2_ws/src/swarm_basics/config/coverage_results.png"
        self.save_path = "/root/ros2_ws/src/results/coverage_results.png" 

        # === GRID SETUP ===
        self.env_min = -7   
        self.env_max = 7
        self.grid_size = 1.0  # 1x1 m cells
        self.cells = [(x, y) for x in range(self.env_min, self.env_max)
                              for y in range(self.env_min, self.env_max)]
        self.visited = set()

        # === ROBOT TRAJECTORIES ===
        self.trajectories = defaultdict(list)
        self.traj_prev_pos = {}             # robot -> last (x, y) for path length
        self.cumulative_path_length = defaultdict(float)

        # === CSV LOGGING ===
        from swarm_basics.run_utils import CONTACT_COOLDOWN_SEC, get_run_dir
        self.csv_dir = get_run_dir()
        self.ros_start_time = self.get_clock().now()  # ROS simulation time

        # trajectories.csv – high-frequency pose stream
        self.traj_file = open(self.csv_dir / 'trajectories.csv', 'w', newline='')
        self.traj_writer = csv.writer(self.traj_file)
        self.traj_writer.writerow(['timestamp', 'elapsed_sec', 'robot_id', 'x', 'y', 'yaw'])

        # events.csv – discrete events: CELL_VISITED / ZONE / BUMP /
        # CONTACT_BEGIN + CONTACT_END.  Contacts are logged ONE PAIR PER
        # EPISODE (not per physics message): a sustained press produces two
        # rows no matter how long it lasts.  The raw per-message stream goes
        # to contacts_raw.csv only when log_raw_contacts:=true.
        self.events_file = open(self.csv_dir / 'events.csv', 'w', newline='')
        self.events_writer = csv.writer(self.events_file)
        self.events_writer.writerow(['timestamp', 'elapsed_sec', 'robot_id', 'event_type', 'details'])

        self.raw_contacts_file = None
        self.raw_contacts_writer = None
        if self.log_raw_contacts:
            self.raw_contacts_file = open(
                self.csv_dir / 'contacts_raw.csv', 'w', newline='')
            self.raw_contacts_writer = csv.writer(self.raw_contacts_file)
            self.raw_contacts_writer.writerow(
                ['timestamp', 'elapsed_sec', 'robot_id', 'event_type', 'details'])

        # Contact episodes: (robot, other) -> open episode.  Same cooldown as
        # bump_counter/social_event_logger => CONTACT_END count == BUMP count.
        self.contact_cooldown = CONTACT_COOLDOWN_SEC
        self.contact_active = {}   # key -> {'other','start_elapsed','samples'}
        self.contact_last = {}     # key -> (elapsed, stamp) of last message
        self.create_timer(1.0, self._prune_contacts)

        # Bump counting per robot (for summary)
        self.total_bumps = defaultdict(int)

        # === SUBSCRIPTIONS ===
        # Global ground-truth poses for all robots
        self.create_subscription(
            TFMessage,
            f'/world/{WORLD_NAME}/dynamic_pose/info',
            self.pose_callback,
            10,
        )

        # Per-robot zone detections & bump counts (robot_0..robot_9)
        for ns in self.robot_namespaces:
            self.create_subscription(
                String, f'/{ns}/detected_zones',
                lambda msg, ns=ns: self.zone_callback(ns, msg), 10)
            self.create_subscription(
                UInt32, f'/{ns}/bump_count',
                lambda msg, ns=ns: self.bump_callback(ns, msg), 10)
            # Raw contact sensor — detailed collision logging
            contact_topic = f"/world/{WORLD_NAME}/model/{ns}/link/{ns}/base_footprint/sensor/contact_sensor/contact"
            self.create_subscription(
                Contacts, contact_topic,
                lambda msg, ns=ns: self.contact_callback(ns, msg), 10)

        # === PLOT SETUP ===
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("Multi-Robot Global Coverage (Ignition ground truth)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)
        # (Agg backend: no GUI window; the coverage PNG is saved periodically)
        self._last_save = 0.0

        # === TIMER FOR UPDATES ===
        if self.dynamic_plots:
            self.timer = self.create_timer(0.5, self.update_plot)
        else:
            self.timer = None
            self.get_logger().info(
                'dynamic_plots=false — live redraws disabled; coverage_map.png '
                'and summary/coverage CSVs will be written once at shutdown.')

    def pose_callback(self, msg: TFMessage):
        """Handle poses from /world/.../dynamic_pose/info (TFMessage)."""
        for t in msg.transforms:
            name = t.child_frame_id
            if not name.startswith('robot_'):
                continue  # skip non-robot entities
            if '/' in name:  # skip any sublink like robot_0/base_link
                continue    

            # extract position (global)
            x = t.transform.translation.x
            y = t.transform.translation.y

            # extract yaw from quaternion
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            self.trajectories[name].append((x, y))

            # track cumulative path length
            if name in self.traj_prev_pos:
                dx = x - self.traj_prev_pos[name][0]
                dy = y - self.traj_prev_pos[name][1]
                self.cumulative_path_length[name] += math.hypot(dx, dy)
            self.traj_prev_pos[name] = (x, y)

            # --- CSV: log trajectory point ---
            now = self.get_clock().now()
            stamp = now.nanoseconds / 1e9
            elapsed = (now - self.ros_start_time).nanoseconds / 1e9
            self.traj_writer.writerow([
                f'{stamp:.3f}',
                f'{elapsed:.3f}',
                name,
                f'{x:.6f}',
                f'{y:.6f}',
                f'{yaw:.4f}',
            ])
            self.traj_file.flush()

            # check which grid cell the robot visited
            for idx, (cx, cy) in enumerate(self.cells):
                if idx not in self.visited:
                    if cx <= x < cx + self.grid_size and cy <= y < cy + self.grid_size:
                        self.visited.add(idx)
                        self.get_logger().info(f"{name} visited cell {idx} at ({cx},{cy})")
                        # --- CSV: log cell-visit event ---
                        self.events_writer.writerow([
                            f'{stamp:.3f}',
                            f'{elapsed:.3f}',
                            name,
                            'CELL_VISITED',
                            f'cell_{cx}_{cy}',
                        ])
                        self.events_file.flush()
                        break

    def zone_callback(self, robot_ns, msg: String):
        """Log detected zone changes (CLEAR/LEFT/RIGHT/CORNER)."""
        now = self.get_clock().now()
        stamp = now.nanoseconds / 1e9
        elapsed = (now - self.ros_start_time).nanoseconds / 1e9
        self.events_writer.writerow([
            f'{stamp:.3f}', f'{elapsed:.3f}', robot_ns, 'ZONE', msg.data,
        ])
        self.events_file.flush()

    def bump_callback(self, robot_ns, msg: UInt32):
        """Log bump events and track count for summary."""
        now = self.get_clock().now()
        stamp = now.nanoseconds / 1e9
        elapsed = (now - self.ros_start_time).nanoseconds / 1e9
        self.total_bumps[robot_ns] = int(msg.data)
        self.events_writer.writerow([
            f'{stamp:.3f}', f'{elapsed:.3f}', robot_ns, 'BUMP', f'count={msg.data}',
        ])
        self.events_file.flush()

    def contact_callback(self, robot_ns, msg: Contacts):
        """Log contacts as EPISODES, not per physics message.

        Same episode rule as bump_counter/social_event_logger (shared
        CONTACT_COOLDOWN_SEC): CONTACT_BEGIN row when contact appears (with
        the contact position), CONTACT_END row when the pair has been silent
        for the cooldown (with duration + raw sample count).  The unthrottled
        stream is only written to contacts_raw.csv with log_raw_contacts:=true.
        """
        now = self.get_clock().now()
        stamp = now.nanoseconds / 1e9
        elapsed = (now - self.ros_start_time).nanoseconds / 1e9
        for c in msg.contacts:
            col1 = c.collision1.name if hasattr(c.collision1, 'name') else str(c.collision1)
            col2 = c.collision2.name if hasattr(c.collision2, 'name') else str(c.collision2)
            is1_me = robot_ns in col1
            is2_me = robot_ns in col2
            if is1_me == is2_me:
                continue
            other = col2 if is1_me else col1
            if c.positions:
                x = c.positions[0].x
                y = c.positions[0].y
                z = c.positions[0].z
            else:
                x = y = z = 0.0

            if self.log_raw_contacts:
                self.raw_contacts_writer.writerow([
                    f'{stamp:.3f}', f'{elapsed:.3f}', robot_ns,
                    'CONTACT', f'{other},{x:.4f},{y:.4f},{z:.4f}'])
                self.raw_contacts_file.flush()

            key = (robot_ns, other)
            last = self.contact_last.get(key)
            if last is None or (elapsed - last[0]) > self.contact_cooldown:
                if last is not None:
                    self._close_contact(key)   # stale episode (gap before prune)
                self.contact_active[key] = {
                    'other': other, 'start_elapsed': elapsed, 'samples': 0}
                self.events_writer.writerow([
                    f'{stamp:.3f}', f'{elapsed:.3f}', robot_ns, 'CONTACT_BEGIN',
                    f'other={other.split("::")[0]} x={x:.4f} y={y:.4f} z={z:.4f}'])
                self.events_file.flush()
            self.contact_active[key]['samples'] += 1
            self.contact_last[key] = (elapsed, stamp)

    def _close_contact(self, key):
        """Emit the CONTACT_END row for an episode and forget it."""
        ep = self.contact_active.pop(key, None)
        last = self.contact_last.pop(key, None)
        if ep is None or last is None:
            return
        end_elapsed, end_stamp = last
        dur = end_elapsed - ep['start_elapsed']
        self.events_writer.writerow([
            f'{end_stamp:.3f}', f'{end_elapsed:.3f}', key[0], 'CONTACT_END',
            f"other={ep['other'].split('::')[0]} dur={dur:.1f}s samples={ep['samples']}"])
        self.events_file.flush()

    def _prune_contacts(self):
        """Close episodes that have been silent for the shared cooldown."""
        now_elapsed = (self.get_clock().now() - self.ros_start_time).nanoseconds / 1e9
        for key, (last_elapsed, _) in list(self.contact_last.items()):
            if (now_elapsed - last_elapsed) > self.contact_cooldown:
                self._close_contact(key)

    def _flush_contacts(self):
        """Close all open episodes (shutdown path)."""
        for key in list(self.contact_last.keys()):
            self._close_contact(key)

    def _save_png(self, path, dpi=None):
        """Save the figure as a PNG, then strip the alpha channel (RGBA->RGB).

        matplotlib always writes RGBA PNGs; VS Code's built-in image preview can
        fail to render those.  Browsers are fine either way, so convert to RGB
        to guarantee the coverage grid is visible everywhere.
        """
        self.fig.savefig(path, dpi=dpi)
        im = Image.open(path).convert('RGB')
        im.save(path)

    def _write_coverage_final_csv(self):
        """Write the full 14x14 visited-grid to coverage_final.csv."""
        grid_path = self.csv_dir / 'coverage_final.csv'
        with open(grid_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['cell_x', 'cell_y', 'visited'])
            for idx, (cx, cy) in enumerate(self.cells):
                w.writerow([cx, cy, 1 if idx in self.visited else 0])

    def _write_summary_csv(self):
        """Write current coverage/path/bump stats to summary.csv."""
        summary_path = self.csv_dir / 'summary.csv'
        visited = len(self.visited)
        total = len(self.cells)
        pct = (visited / total) * 100 if total > 0 else 0.0
        duration = (self.get_clock().now() - self.ros_start_time).nanoseconds / 1e9
        with open(summary_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['metric', 'value'])
            w.writerow(['total_robots', len(self.trajectories)])
            w.writerow(['duration_sec', f'{duration:.1f}'])
            w.writerow(['total_cells', total])
            w.writerow(['visited_cells', visited])
            w.writerow(['coverage_pct', f'{pct:.2f}'])
            total_path = sum(self.cumulative_path_length.values())
            w.writerow(['total_path_length_m', f'{total_path:.2f}'])
            all_bumps = sum(self.total_bumps.values())
            w.writerow(['total_bumps', all_bumps])
            for ns in self.trajectories:
                pts = len(self.trajectories[ns])
                dist = self.cumulative_path_length[ns]
                bumps = self.total_bumps.get(ns, 0)
                w.writerow([f'{ns}_path_points', pts])
                w.writerow([f'{ns}_path_length_m', f'{dist:.2f}'])
                w.writerow([f'{ns}_bumps', bumps])
            # Goal outcomes written by the goal driver (fixed_goals /
            # random_goals) -> merged here so every arm's summary carries
            # success/failure counts (goals_sent/succeeded/failed + scans).
            goal_stats = self.csv_dir / 'goal_stats.csv'
            if goal_stats.exists():
                try:
                    with open(goal_stats, newline='') as gf:
                        for row in csv.DictReader(gf):
                            rid = (row.get('robot_id') or '').strip()
                            if not rid:
                                continue
                            w.writerow([f'goals_sent_{rid}',
                                        row.get('goals_sent')])
                            w.writerow([f'goals_succeeded_{rid}',
                                        row.get('goals_succeeded')])
                            w.writerow([f'goals_failed_{rid}',
                                        row.get('goals_failed')])
                            w.writerow([f'goals_scan_{rid}',
                                        row.get('scan_goals')])
                except OSError:
                    pass

    def update_plot(self):
        """Redraw robot trajectories and visited cells."""
        self.ax.clear()
        self.ax.set_title("Multi-Robot Global Coverage (Ignition ground truth)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)

        # Plot grid cells
        for idx, (cx, cy) in enumerate(self.cells):
            color = 'green' if idx in self.visited else 'red'
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size,
                             facecolor=color, edgecolor='black', alpha=0.5)
            self.ax.add_patch(rect)

        # Plot trajectories
        for ns, traj in self.trajectories.items():
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax.plot(xs, ys, label=ns)

        if self.trajectories:
            self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells)} cells",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )
        # headless: render offscreen and periodically save the coverage PNG so
        # the map is still watchable as a file (no unstoppable GUI window).
        plt.draw()
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_save > 2.0:
            self._last_save = now
            self._save_png(self.save_path)
            self._save_png(self.csv_dir / 'coverage_map.png', dpi=150)
            # Refresh summary/coverage CSVs alongside the PNG every 2s so an
            # interrupted run (Ctrl-C, plotter crash on shutdown) never loses
            # these files.
            self._write_coverage_final_csv()
            self._write_summary_csv()

    def save_final_plot(self):
        """Save final coverage plot."""
        self.ax.clear()
        self.ax.set_title("Final Global Coverage Map")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)

        for idx, (cx, cy) in enumerate(self.cells):
            color = 'green' if idx in self.visited else 'red'
            rect = Rectangle((cx, cy), self.grid_size, self.grid_size,
                             facecolor=color, edgecolor='black', alpha=0.5)
            self.ax.add_patch(rect)

        for ns, traj in self.trajectories.items():
            if len(traj) > 1:
                xs, ys = zip(*traj)
                self.ax.plot(xs, ys, label=ns)

        if self.trajectories:
            self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells)} cells",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        self._save_png(self.save_path)
        self._save_png(self.csv_dir / 'coverage_map.png', dpi=150)

        # --- CSV: final coverage grid + summary (already kept fresh by the
        # periodic save, this is the last write) ---
        self._write_coverage_final_csv()
        self._write_summary_csv()

        # --- close any contact episode still open at shutdown, then the CSVs
        # (an episode in progress when Ctrl-C hits would otherwise lose its
        # duration/sample summary) ---
        self._flush_contacts()
        self.traj_file.close()
        self.events_file.close()
        if self.raw_contacts_file is not None:
            self.raw_contacts_file.close()

        visited = len(self.visited)
        total = len(self.cells)
        pct = (visited / total) * 100 if total > 0 else 0.0
        self.get_logger().info(
            f"Final plot saved. {visited}/{total} cells visited ({pct:.1f}%). "
            f"CSVs in {self.csv_dir}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down coverage plotter.")
    finally:
        # The shutdown signal arrives more than once (ros2 launch forwards
        # SIGINT to its children in addition to the one we already received).
        # A second KeyboardInterrupt would abort the final save mid-render —
        # observed: KeyboardInterrupt inside save_final_plot lost
        # coverage_map.png, summary.csv and coverage_final.csv entirely.
        # Ignore further SIGINTs while the final save runs.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.save_final_plot()
        except Exception as exc:  # save what we can, never die silently
            node.get_logger().error(f"Final save failed: {exc!r}")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
