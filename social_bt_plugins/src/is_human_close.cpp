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

#include "is_robot_close.hpp"

/**
 * IsHumanClose — custom BT condition node for social navigation.
 *
 * Subscribes to the robot's `human_close` (Bool) and `human_angle` (Float32)
 * topics (namespaced, e.g. /robot_0/human_close, /robot_0/human_angle) and
 * decides whether THIS robot should yield to a human:
 *
 *   * yield when a human is close AND inside the block cone
 *     (±block_angle_deg, default 20°),
 *   * keep yielding up to yield_max_s (default 4 s), then stop,
 *   * enter a cooldown (cooldown_s, default 4 s) before it can yield again.
 *
 * SIDE humans (outside the cone) do NOT trigger the hard yield — they are left
 * to the velocity_adaptor, which smoothly slows the robot instead.  Frontal
 * humans get the hard stop as a safety net.
 *
 * Writes the decision into blackboard `human_close` (true while actually
 * yielding) and `human_angle` for the tree's BlackboardCheckBool.  Publishes
 * lifecycle events on `bt_social_event` for the CSV logger:
 * HUMAN_YIELD_START / HUMAN_YIELD_END / HUMAN_SUPPRESS_START / HUMAN_SUPPRESS_END.
 *
 * Runs its OWN rclcpp node + executor + thread so subscriptions are guaranteed
 * to receive callbacks (Nav2's internal blackboard "node" is not reliably spun).
 */

namespace social_bt
{

class IsHumanClose : public BT::ConditionNode
{
public:
  IsHumanClose(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf),
    human_close_(false), human_angle_(0.0),
    yielding_(false), suppressing_(false),
    yield_started_s_(0.0), suppress_until_s_(0.0),
    block_angle_deg_(20.0), yield_max_s_(4.0), cooldown_s_(4.0)
  {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "is_human_close_node", node->get_namespace(),
        rclcpp::NodeOptions());
      sub_close_ = local_node_->create_subscription<std_msgs::msg::Bool>(
        "human_close", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          human_close_ = msg->data;
        });
      sub_angle_ = local_node_->create_subscription<std_msgs::msg::Float32>(
        "human_angle", rclcpp::QoS(10),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          human_angle_ = msg->data;
        });
      event_pub_ = local_node_->create_publisher<std_msgs::msg::String>(
        "bt_social_event", 10);
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("IsHumanClose"),
                  "IsHumanClose: subscribed to %s/human_close + human_angle "
                  "(block=%.0f deg, yield=%.0fs, cooldown=%.0fs)",
                  node->get_namespace(), block_angle_deg_, yield_max_s_,
                  cooldown_s_);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("IsHumanClose"),
                  "blackboard has no 'node' — cannot subscribe to human_close");
    }
  }

  ~IsHumanClose()
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

    // Yield only for humans close AND inside the forward cone (safety net).
    const bool in_block =
      human_close_ && std::abs(human_angle_) <= block_angle_deg_;

    if (suppressing_ && now >= suppress_until_s_) {
      suppressing_ = false;
      publish_event("HUMAN_SUPPRESS_END");
    }

    const bool should_yield = in_block && !suppressing_;

    if (should_yield && !yielding_) {
      yielding_ = true;
      yield_started_s_ = now;
      publish_event("HUMAN_YIELD_START");
    }

    if (yielding_) {
      if (now - yield_started_s_ >= yield_max_s_) {
        yielding_ = false;
        suppressing_ = true;
        suppress_until_s_ = now + cooldown_s_;
        publish_event("HUMAN_SUPPRESS_START");
      }
    }

    if (!should_yield && yielding_) {
      yielding_ = false;
      publish_event("HUMAN_YIELD_END");
    }

    // Feed the tree: human_close true only while we are actually yielding.
    config().blackboard->set("human_close", yielding_);
    config().blackboard->set("human_angle", static_cast<double>(human_angle_));

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
    RCLCPP_INFO(rclcpp::get_logger("IsHumanClose"), "%s", ev.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_close_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_angle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Node::SharedPtr local_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;

  bool human_close_;
  float human_angle_;
  bool yielding_;
  bool suppressing_;
  double yield_started_s_;
  double suppress_until_s_;
  double block_angle_deg_;
  double yield_max_s_;
  double cooldown_s_;
};

}  // namespace social_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_bt::IsHumanClose>("IsHumanClose");
  factory.registerNodeType<social_bt::IsRobotClose>("IsRobotClose");
}
