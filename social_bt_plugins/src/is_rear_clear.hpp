// IsRearClear — custom BT condition node: is the space BEHIND the rover
// clear enough to reverse safely?
//
// Subscribes to the robot's `lidar/scan_clean` (sensor_msgs/LaserScan) and
// reports SUCCESS when the nearest obstacle in the REAR sector
// (|bearing| >= 180 - rear_half_deg, i.e. behind the rover) is farther than
// clear_dist (m).  Gates the LLM "back" action so the rover never reverses
// into a wall, human or other rover.
//
// Used inside a ReactiveSequence in ai_nav.xml, so it is re-checked on every
// tick WHILE the BackUp runs: if an obstacle appears behind mid-reverse the
// sequence halts the backup immediately.
//
// Runs its OWN rclcpp node + executor + thread (like IsRobotClose) so the
// scan subscription reliably receives callbacks.
//
// Registered from is_human_close.cpp (single BT_REGISTER_NODES block).

#ifndef IS_REAR_CLEAR_HPP_
#define IS_REAR_CLEAR_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

namespace social_bt
{

class IsRearClear : public BT::ConditionNode
{
public:
  IsRearClear(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf),
    min_rear_(std::numeric_limits<double>::infinity()), got_scan_(false)
  {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "is_rear_clear_node", node->get_namespace(), rclcpp::NodeOptions());
      sub_scan_ = local_node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar/scan_clean", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {
          double min_r = std::numeric_limits<double>::infinity();
          const double amin = m->angle_min, inc = m->angle_increment;
          for (size_t i = 0; i < m->ranges.size(); ++i) {
            const float r = m->ranges[i];
            if (!std::isfinite(r) || r <= 0.0f) {
              continue;
            }
            const double deg = (amin + static_cast<double>(i) * inc) *
                               180.0 / M_PI;
            if (std::fabs(deg) >= 150.0) {   // rear sector (see rear_half_deg)
              min_r = std::min(min_r, static_cast<double>(r));
            }
          }
          min_rear_ = min_r;
          got_scan_ = true;
        });
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("IsRearClear"),
                  "IsRearClear: subscribed to %s/lidar/scan_clean (rear "
                  "sector, clear_dist=0.8 m)",
                  node->get_namespace());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("IsRearClear"),
                  "blackboard has no 'node' — rear check disabled");
    }
  }

  ~IsRearClear()
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_ && spin_thread_->joinable()) {
      spin_thread_->join();
    }
  }

  BT::NodeStatus tick() override
  {
    if (!local_node_) {
      return BT::NodeStatus::FAILURE;
    }
    double clear_dist = 0.8;
    getInput("clear_dist", clear_dist);
    // No scan data yet (or lidar missing) -> never reverse blind.
    if (!got_scan_) {
      return BT::NodeStatus::FAILURE;
    }
    return (min_rear_ >= clear_dist) ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("clear_dist", 0.8,
                            "min free distance behind the rover (m)"),
    };
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Node::SharedPtr local_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;

  double min_rear_;   // nearest obstacle in the rear sector (m)
  bool got_scan_;     // whether a scan has been received yet
};

}  // namespace social_bt

#endif  // IS_REAR_CLEAR_HPP_
