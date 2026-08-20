// IsRobotClose — custom BT condition node for robot-robot social navigation.
//
// Subscribes to the robot's `robot_close` (Bool) and `robot_angle` (Float32)
// topics, published by swarm_basics robot_proximity (namespaced, e.g.
// /robot_0/robot_close, /robot_0/robot_angle).  On every tick it decides
// whether THIS robot should yield to another robot:
//
//   * yield when a robot is close AND its bearing is inside the block cone
//     (±block_angle_deg, default 30°),
//   * keep yielding up to yield_max_s (default 7 s), then stop,
//   * enter a cooldown (cooldown_s, default 6 s) before it can yield again.
//
// Writes the decision into blackboard keys `robot_close` (bool, true while
// yielding) and `robot_angle` (float) for the tree's BlackboardCheckBool.
// Publishes lifecycle events on `bt_social_event` for logging:
// ROBOT_YIELD_START / ROBOT_YIELD_END / ROBOT_SUPPRESS_START / ROBOT_SUPPRESS_END.
//
// Like IsHumanClose it runs its OWN rclcpp node + executor + thread so the
// subscriptions are guaranteed to receive callbacks (Nav2's internal blackboard
// "node" is not reliably spun).
//
// Registered from is_human_close.cpp (single BT_REGISTER_NODES block).

#ifndef IS_ROBOT_CLOSE_HPP_
#define IS_ROBOT_CLOSE_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace social_bt
{

class IsRobotClose : public BT::ConditionNode
{
public:
  IsRobotClose(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf),
    robot_close_(false), robot_angle_(0.0),
    yielding_(false), suppressing_(false),
    yield_started_s_(0.0), suppress_until_s_(0.0),
    block_angle_deg_(30.0), yield_max_s_(7.0), cooldown_s_(6.0)
  {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "is_robot_close_node", node->get_namespace(),
        rclcpp::NodeOptions());
      sub_close_ = local_node_->create_subscription<std_msgs::msg::Bool>(
        "robot_close", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          robot_close_ = msg->data;
        });
      sub_angle_ = local_node_->create_subscription<std_msgs::msg::Float32>(
        "robot_angle", rclcpp::QoS(10),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          robot_angle_ = msg->data;
        });
      event_pub_ = local_node_->create_publisher<std_msgs::msg::String>(
        "bt_social_event", 10);
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("IsRobotClose"),
                  "IsRobotClose: subscribed to %s/robot_close + robot_angle "
                  "(block=%.0f deg, yield=%.0fs, cooldown=%.0fs)",
                  node->get_namespace(), block_angle_deg_, yield_max_s_,
                  cooldown_s_);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("IsRobotClose"),
                  "blackboard has no 'node' — cannot subscribe to robot_close");
    }
  }

  ~IsRobotClose()
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
    rclcpp::Node::SharedPtr node;
    double now = 0.0;
    if (config().blackboard->get("node", node) && node) {
      now = node->now().seconds();
    }

    const bool in_block =
      robot_close_ && std::abs(robot_angle_) <= block_angle_deg_;

    // Cooldown: let the robot resume once the suppression window elapses.
    if (suppressing_ && now >= suppress_until_s_) {
      suppressing_ = false;
      publish_event("ROBOT_SUPPRESS_END");
    }

    // Yield decision: close AND inside cone AND not cooling down.
    const bool should_yield = in_block && !suppressing_;

    if (should_yield && !yielding_) {
      yielding_ = true;
      yield_started_s_ = now;
      publish_event("ROBOT_YIELD_START");
    }

    if (yielding_) {
      // Give the other robot a fixed window, then stop + cool down.
      if (now - yield_started_s_ >= yield_max_s_) {
        yielding_ = false;
        suppressing_ = true;
        suppress_until_s_ = now + cooldown_s_;
        publish_event("ROBOT_SUPPRESS_START");
      }
    }

    if (!should_yield && yielding_) {
      yielding_ = false;
      publish_event("ROBOT_YIELD_END");
    }

    // Feed the tree: robot_close true while we are actually yielding.
    config().blackboard->set("robot_close", yielding_);
    config().blackboard->set("robot_angle", robot_angle_);

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  void publish_event(const std::string & ev)
  {
    if (!event_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = ev;
    event_pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("IsRobotClose"), "%s", ev.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_close_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_angle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Node::SharedPtr local_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;

  bool robot_close_;
  float robot_angle_;
  bool yielding_;
  bool suppressing_;
  double yield_started_s_;
  double suppress_until_s_;
  double block_angle_deg_;
  double yield_max_s_;
  double cooldown_s_;
};

}  // namespace social_bt

#endif  // IS_ROBOT_CLOSE_HPP_
