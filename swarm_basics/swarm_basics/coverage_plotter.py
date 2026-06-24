import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from collections import defaultdict


class CoveragePlotter(Node):
    def __init__(self):
        super().__init__('coverage_plotter')

        # === CONFIGURATION ===
        self.robot_namespaces = [f"robot_{i}" for i in range(10)]
        #self.save_path = "/home/ecem/ros2_ws/src/swarm_basics/config/coverage_results.png"
        self.save_path = "/root/ros2_ws/coverage_results.png" 

        # === GRID SETUP ===
        self.env_min = -7   
        self.env_max = 7
        self.grid_size = 1.0  # 1x1 m cells
        self.cells = [(x, y) for x in range(self.env_min, self.env_max)
                              for y in range(self.env_min, self.env_max)]
        self.visited = set()

        # === ROBOT TRAJECTORIES ===
        self.trajectories = defaultdict(list)

        # === SUBSCRIPTION ===
        # We only need one subscription for all robots (global positions)
        self.create_subscription(
            TFMessage,
            #'/world/u_corridor/dynamic_pose/info',
            #'/world/random_world/dynamic_pose/info',
            '/world/custom_corridor/dynamic_pose/info',
            self.pose_callback,
            10,
        )

        # === PLOT SETUP ===
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("Multi-Robot Global Coverage (Ignition ground truth)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(self.env_min, self.env_max)
        self.ax.set_ylim(self.env_min, self.env_max)
        plt.ion()
        plt.show()

        # === TIMER FOR UPDATES ===
        self.timer = self.create_timer(0.5, self.update_plot)

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

            self.trajectories[name].append((x, y))
            # self.get_logger().info(f"{name}: x={x:.2f}, y={y:.2f}")

            # check which grid cell the robot visited
            for idx, (cx, cy) in enumerate(self.cells):
                if idx not in self.visited:
                    if cx <= x < cx + self.grid_size and cy <= y < cy + self.grid_size:
                        self.visited.add(idx)
                        self.get_logger().info(f"{name} visited cell {idx} at ({cx},{cy})")
                        break

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

        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells)} cells",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )
        plt.draw()
        plt.pause(0.01)

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

        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.text(
            0.05, 0.95,
            f"Visited {len(self.visited)}/{len(self.cells)} cells",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        self.fig.savefig(self.save_path)
        self.get_logger().info(
            f"Final plot saved. {len(self.visited)}/{len(self.cells)} cells visited."
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_final_plot()
        node.get_logger().info("Shutting down coverage plotter.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
