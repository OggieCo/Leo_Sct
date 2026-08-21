// Arc — custom BT action node for a smooth, constant-curvature curved maneuver.
//
// Drives the rover along a circular arc: it publishes BOTH linear.x and
// angular.z cmd_vel together and integrates the heading change from odometry,
// stopping once the robot has turned `arc_angle` radians.  This is a true
// "turn while driving" maneuver — NO in-place spin — so it works on the lab
// rovers that have adhesive tape on their tyres (they cannot zero-turn).
//
// The arc always turns LEFT (positive angular).  In the head-on test the two
// rovers face OPPOSITE ways, so "left" is opposite in world frame and the
// arcs naturally DIVERGE them — the same geometry as the old Spin+Drive.
//
// Uses its OWN rclcpp node + executor + thread (like IsRobotClose) so the
// odometry subscription and cmd_vel publisher are reliably spun.
//
// Registered from is_human_close.cpp (single BT_REGISTER_NODES block).

#ifndef SOCIAL_BT__ARC_HPP_
#define SOCIAL_BT__ARC_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace social_bt
{

class Arc : public BT::ActionNodeBase
{
public:
  Arc(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(name, conf),
    started_(false), initial_yaw_(0.0), odom_yaw_(0.0), odom_ok_(false),
    start_time_s_(0.0), linear_(0.0), angular_(0.0), arc_angle_(0.0),
    time_allowance_s_(6.0)
  {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "arc_node", node->get_namespace(), rclcpp::NodeOptions());
      sub_odom_ = local_node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_yaw_ = 2.0 * std::atan2(
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
          odom_ok_ = true;
        });
      pub_cmd_ = local_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 10);
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("Arc"),
                  "Arc: smooth curved maneuver ready (ns=%s)",
                  node->get_namespace());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("Arc"),
                  "blackboard has no 'node' — arc disabled");
    }
  }

  ~Arc()
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_ && spin_thread_->joinable()) {
      spin_thread_->join();
    }
  }

  void halt() override
  {
    stop();
    started_ = false;
    // NOTE: do NOT call BT::ActionNodeBase::halt() — in BT.CPP v3.8 the base
    // TreeNode::halt() has no implementation, so calling it leaves an
    // unresolved symbol and the plugin fails to load.
  }

  BT::NodeStatus tick() override
  {
    if (!local_node_) {
      return BT::NodeStatus::FAILURE;
    }

    rclcpp::Node::SharedPtr node;
    if (!config().blackboard->get("node", node) || !node) {
      return BT::NodeStatus::FAILURE;
    }

    if (!started_) {
      double arc_angle = 1.05, speed = 0.3, radius = 0.95, time_allowance = 6.0;
      std::string direction = "left";
      getInput("arc_angle", arc_angle);
      getInput("speed", speed);
      getInput("radius", radius);
      getInput("time_allowance", time_allowance);
      getInput("direction", direction);
      arc_angle_ = std::fabs(arc_angle);
      time_allowance_s_ = time_allowance;
      linear_ = speed;
      // Direction: fixed left/right, or "away" = turn away from the detected
      // rover (bearing from {robot_angle}, deg, + = left).  For head-on
      // encounters both rovers see each other on the SAME side, so turning
      // away makes them turn in opposite world directions -> they diverge.
      const double ang_mag = speed / radius;
      if (direction == "right") {
        angular_ = -ang_mag;
      } else if (direction == "away") {
        double robot_angle = 0.0;
        config().blackboard->get("robot_angle", robot_angle);
        angular_ = (robot_angle > 0.0) ? -ang_mag : ang_mag;
      } else {  // "left" (default)
        angular_ = ang_mag;
      }

      if (!odom_ok_) {
        return BT::NodeStatus::RUNNING;  // wait for odometry before starting
      }
      initial_yaw_ = odom_yaw_;
      start_time_s_ = node->now().seconds();
      started_ = true;
      RCLCPP_INFO(rclcpp::get_logger("Arc"),
                  "starting arc: turn %.2f rad at r=%.2f, v=%.2f (%s)",
                  arc_angle_, radius, speed, direction.c_str());
    }

    if (node->now().seconds() - start_time_s_ > time_allowance_s_) {
      stop();
      started_ = false;
      RCLCPP_WARN(rclcpp::get_logger("Arc"), "Arc time allowance exceeded");
      return BT::NodeStatus::FAILURE;
    }

    // Publish the arc velocity (drive AND turn simultaneously).
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_;
    cmd.angular.z = angular_;
    pub_cmd_->publish(cmd);

    // Heading change from odometry (wrapped to [-pi, pi]).
    double dyaw = odom_yaw_ - initial_yaw_;
    while (dyaw > M_PI) {
      dyaw -= 2.0 * M_PI;
    }
    while (dyaw < -M_PI) {
      dyaw += 2.0 * M_PI;
    }
    if (std::fabs(dyaw) >= arc_angle_) {
      stop();
      started_ = false;
      RCLCPP_INFO(rclcpp::get_logger("Arc"),
                  "Arc complete (turned %.2f rad)", std::fabs(dyaw));
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("arc_angle", 1.05, "total turn angle (rad)"),
      BT::InputPort<double>("radius", 0.95, "arc radius (m)"),
      BT::InputPort<double>("speed", 0.3, "linear speed (m/s)"),
      BT::InputPort<double>("time_allowance", 6.0, "max seconds"),
      BT::InputPort<std::string>(
        "direction", "left", "left | right | away (turn away from detected rover)"),
    };
  }

private:
  void stop()
  {
    if (pub_cmd_) {
      geometry_msgs::msg::Twist zero;
      pub_cmd_->publish(zero);
    }
  }

  rclcpp::Node::SharedPtr local_node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;

  bool started_;
  double initial_yaw_;
  double odom_yaw_;
  bool odom_ok_;
  double start_time_s_;
  double linear_;
  double angular_;
  double arc_angle_;
  double time_allowance_s_;
};

}  // namespace social_bt

#endif  // SOCIAL_BT__ARC_HPP_
