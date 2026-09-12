import os
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from swarm_basics.sct import SCT
from ament_index_python.packages import get_package_share_directory
import time, random, math

from nav_msgs.msg import Odometry
# from math import atan2, sqrt, pi


class RobotSupervisor(Node):
    def __init__(self):
        super().__init__('robot_supervisor_3_movements')

        # Load SCT YAML
        config_path = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'sup_gpt.yaml'
        )
        if not os.path.exists(config_path):
            self.get_logger().error(f"YAML file not found: {config_path}")
            rclpy.shutdown()
            return

        self.sct = SCT(config_path)
        self.target_offset = 0.0
        self.target_distance = float('inf')
        self.obstacle_zones = []

        # Ensure all events have a callback entry
        for ev_name, ev_id in self.sct.EV.items():
            if ev_id not in self.sct.callback:
                self.sct.callback[ev_id] = {"callback": None}

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber
        self.sub = self.create_subscription(String, 'detected_zones', self.zone_callback, 10)


        # SCT callbacks for UCEs
        self.sct.add_callback(self.sct.EV['EV_obstacle_front'],None,self.middle_check,None) # EV_S0
        self.sct.add_callback(self.sct.EV['EV_path_clear'],None,self.clear_path_check,None) # EV_S1
        self.sct.add_callback(self.sct.EV['EV_obstacle_left'],None,self.left_check,None)    # EV_S2
        self.sct.add_callback(self.sct.EV['EV_obstacle_right'],None,self.right_check,None) # EV_S3
        # self.sct.add_callback(self.sct.EV['EV_S4'],None,self.red_check,None)
        # self.sct.add_callback(self.sct.EV['EV_S5'],None,self.blue_check,None)

        # Patch make_transition to log supervisor transitions
        original_make_transition = self.sct.make_transition
        def logged_make_transition(ev):
            ev_name = list(self.sct.EV.keys())[ev]
            # self.get_logger().info(f"--- Applying event: {ev_name} (index {ev}) ---")
            old_states = self.sct.sup_current_state.copy()
            original_make_transition(ev)

        self.sct.make_transition = logged_make_transition

        self.timer = self.create_timer(0.1, self.timer_callback)


    def zone_callback(self, msg):
        self.obstacle_zones = [z.strip() for z in msg.data.split(',') if z.strip()]
        self.get_logger().info(f'ZONE: {self.obstacle_zones}')

        for z in self.obstacle_zones:
            parts = z.split(",")
            if len(parts) == 3:
                try:
                    self.target_offset = float(parts[1])
                    self.target_distance = float(parts[2])
                    break
                except ValueError:
                    continue



    # -------------------------------
    # SCT input check functions
    # -------------------------------
    def clear_path_check(self, sup_data):
        depth_obstacles = {'LEFT', 'RIGHT', 'CORNER'}
        return not any(zone in depth_obstacles for zone in self.obstacle_zones)

    def middle_check(self, sup_data):
        return 'CORNER' in self.obstacle_zones
            
    def left_check(self, sup_data):
        return 'LEFT' in self.obstacle_zones
           
    def right_check(self, sup_data):
        return 'RIGHT' in self.obstacle_zones

    # def red_check(self, sup_data):
    #     return 'RED' in self.obstacle_zones

    # def blue_check(self, sup_data):
    #     return 'BLUE' in self.obstacle_zones            


    # -------------------------------
    # Robot motion
    # -------------------------------
    def publish_twist(self, ev_name: str):
        twist = Twist()

        if ev_name == "EV_random_walk": # EV_V1      
            self.get_logger().info("Supervisor decision: RANDOM WALK")
            twist.linear.x = random.uniform(0.01, 1.0)
            twist.angular.z = random.uniform(-1.0, 1.0)
                
        elif ev_name == "EV_full_rotate": # EV_V0      
            self.get_logger().info("Supervisor decision: OBSTACLE")
            twist.linear.x = 0.0
            twist.angular.z = 1.0
            time.sleep(0.1) 

        elif ev_name == "EV_clockwise_turn": # EV_V2    
            self.get_logger().info("Supervisor decision: CW")
            twist.linear.x = 0.0
            twist.angular.z = -0.5

        elif ev_name == "EV_counterclockwise_turn":  # EV_V3  
            self.get_logger().info("Supervisor decision: CCW")
            twist.linear.x = 0.0  
            twist.angular.z = 0.5
        
        elif ev_name == "EV_move_forward":   # EV_V4  
            self.get_logger().info("Supervisor decision: FORWARD")
            twist.linear.x = 0.5
            twist.angular.z = 0.0  

        elif ev_name == "EV_move_backward":  # EV_V5   
            self.get_logger().info("Supervisor decision: BACKWARD")
            twist.linear.x = -0.5
            twist.angular.z = 0.0      


        # elif ev_name == "EV_V4": # RED
        #     if self.target_distance > 1.0 and self.target_distance < 0.5:
        #         twist.linear.x = 0.2
        #         twist.angular.z = -1.0 * self.target_offset
        #         self.get_logger().info(f"Going toward: offset={self.target_offset:.2f}, dist={self.target_distance:.2f}")

        # elif ev_name == "EV_V5": # BLUE  
        #         twist.linear.x = 0.0
        #         twist.angular.z = -1.0 * abs(self.target_offset)
        #         self.get_logger().info(f"Escaping: offset={self.target_offset:.2f}, dist={self.target_distance:.2f}")
           
        else:
            # Unknown or no event -> stop for safety
            twist.linear.x = 0.0
            # self.get_logger().debug(f"Unknown or None event: {ev_name}")

        self.cmd_pub.publish(twist)


    # -------------------------------
    # Timer callback
    # -------------------------------
    def timer_callback(self):
        # Run supervisor logic
        self.sct.input_buffer = []
        ce_exists, ce = self.sct.run_step()

        if ce_exists:
            event_name = [name for name, val in self.sct.EV.items() if val == ce][0]
            # self.get_logger().info(f"Controllable event chosen: {event_name}")
            self.publish_twist(event_name)
   

def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    rclpy.spin(node)

    # cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
