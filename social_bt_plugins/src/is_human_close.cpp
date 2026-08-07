#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <thread>

/**
 * IsHumanClose — custom BT condition node for social navigation.
 *
 * Subscribes to the robot's `human_close` topic (published by
 * image_human_processor, namespaced e.g. /robot_0/human_close) and writes the
 * value into the BT blackboard key `human_close` on every tick.  The social
 * tree (social_nav.xml) reads that key to decide whether to yield.
 *
 * Returns SUCCESS always (it's a blackboard-feeder, not a gate — the tree's
 * ReactiveFallback / BlackboardCheckBool actually decides the behavior).
 */

namespace social_bt
{

class IsHumanClose : public BT::ConditionNode
{
public:
  IsHumanClose(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf), human_close_(false), human_angle_(0.0),
    block_angle_deg_(20.0), yield_max_s_(7.0), cooldown_s_(6.0),
    yielding_(false), yield_start_s_(0.0), suppress_until_s_(0.0)
  {
    getInput<double>("block_angle_deg", block_angle_deg_);
    getInput<double>("yield_max_s", yield_max_s_);
    getInput<double>("cooldown_s", cooldown_s_);
    // Nav2's internal BT node (blackboard "node") is NOT reliably spun, so
    // subscriptions on it never receive callbacks.  Create our OWN node +
    // executor + thread so the human_close subscription is guaranteed to fire.
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "is_human_close_node", node->get_namespace(),
        rclcpp::NodeOptions());
      sub_ = local_node_->create_subscription<std_msgs::msg::Bool>(
        "human_close", rclcpp::QoS(10),   // reliable: matches the rclpy publisher
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          human_close_ = msg->data;
          RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("IsHumanClose"), *local_node_->get_clock(), 1000,
            "IsHumanClose: recv human_close=%d angle=%.1f", human_close_, human_angle_);
        });
      angle_sub_ = local_node_->create_subscription<std_msgs::msg::Float32>(
        "human_angle", rclcpp::QoS(10),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          human_angle_ = msg->data;
        });
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("IsHumanClose"),
                  "IsHumanClose: subscribed to %s/human_close (own executor)",
                  node->get_namespace());
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
    // Raw condition: human close AND roughly in the forward path.
    bool raw = human_close_ && std::fabs(human_angle_) < block_angle_deg_;

    // Use the blackboard node's clock (sim time) for yield/cooldown timing.
    rclcpp::Node::SharedPtr node;
    rclcpp::Clock::SharedPtr clock;
    if (config().blackboard->get("node", node) && node) {
      clock = node->get_clock();
    }
    double now_s = clock ? clock->now().seconds() : 0.0;

    if (raw) {
      if (!yielding_) {
        yielding_ = true;
        yield_start_s_ = now_s;
      }
      // After the FULL yield window, suppress the yield for a cooldown so the
      // rover actually has time to drive AROUND a static human instead of
      // re-yielding forever (the pre-fix oscillation).
      if (suppress_until_s_ <= 0.0 &&
          (now_s - yield_start_s_) >= yield_max_s_) {
        suppress_until_s_ = now_s + cooldown_s_;
        RCLCPP_INFO(rclcpp::get_logger("IsHumanClose"),
                    "yield maxed after %.0f s — suppressing for %.0f s so the rover can pass",
                    yield_max_s_, cooldown_s_);
      }
    } else {
      yielding_ = false;
      suppress_until_s_ = 0.0;
    }

    bool effective = raw;
    if (suppress_until_s_ > 0.0) {
      if (now_s >= suppress_until_s_) {
        suppress_until_s_ = 0.0;
      } else {
        effective = false;  // let FollowPath drive away
      }
    }

    config().blackboard->set("human_close", effective);
    config().blackboard->set("human_angle", human_angle_);

    if (clock) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("IsHumanClose"), *clock, 2000,
        "IsHumanClose: ticking, close=%d angle=%.1f block=%.0f eff=%d%s",
        human_close_, human_angle_, block_angle_deg_, effective,
        suppress_until_s_ > 0.0 ? " [cooldown]" : "");
    }
    // edge-triggered log: proves the effective yield condition actually updates
    if (effective != last_reported_) {
      last_reported_ = effective;
      RCLCPP_INFO(rclcpp::get_logger("IsHumanClose"),
                  "yield (close && |angle|<%.0f) -> %s",
                  block_angle_deg_, effective ? "TRUE" : "false");
    }
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "block_angle_deg", 20.0,
        "Yield only while |human_angle| < this (deg); beyond it the way is clear"),
      BT::InputPort<double>(
        "yield_max_s", 7.0,
        "Max seconds to keep yielding before suppressing (matches tree Timeout)"),
      BT::InputPort<double>(
        "cooldown_s", 6.0,
        "Seconds to suppress the yield after the max so the rover can pass")
    };
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Node::SharedPtr local_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;
  bool human_close_;
  double human_angle_;
  double block_angle_deg_;
  double yield_max_s_;
  double cooldown_s_;
  bool yielding_;
  double yield_start_s_;
  double suppress_until_s_;
  bool last_reported_ = false;
};

}  // namespace social_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_bt::IsHumanClose>("IsHumanClose");
}
