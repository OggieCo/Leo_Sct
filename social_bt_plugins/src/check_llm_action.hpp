// CheckLlmAction — custom BT condition node that feeds the LLM planner's
// decision into the blackboard.
//
// Subscribes to the robot's `llm_action` (String) and `llm_reason` (String)
// topics, published by swarm_basics llm_planner (namespaced).  On every tick
// it copies the latest values into blackboard keys `llm_action` / `llm_reason`
// and returns SUCCESS (it is a pure "refresh" node).
//
// The tree uses these keys to drive behaviour: yield / proceed / arc / stop.
// When no llm_planner is running (topics absent) the keys stay empty and the
// tree falls back to the reactive rules (IsRobotClose / IsHumanClose).
//
// Runs its OWN rclcpp node + executor + thread so subscriptions are guaranteed
// to receive callbacks.
//
// Registered from is_human_close.cpp (single BT_REGISTER_NODES block).

#ifndef CHECK_LLM_ACTION_HPP_
#define CHECK_LLM_ACTION_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <thread>

namespace social_bt
{

class CheckLlmAction : public BT::ConditionNode
{
public:
  CheckLlmAction(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf)
  {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "check_llm_action_node", node->get_namespace(),
        rclcpp::NodeOptions());
      sub_action_ = local_node_->create_subscription<std_msgs::msg::String>(
        "llm_action", rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          llm_action_ = msg->data;
          // Keep the blackboard FRESH even while the tree is parked in the
          // AiYield Wait branch: the AiPath Sequence only re-ticks
          // CheckLlmAction when its running child restarts, so a late
          // "proceed" (e.g. the deterministic yield release) was ignored for
          // up to the whole AiYieldMax window (run_2026-08-24_14-22-59:
          // proceed published at 46.6s, robot resumed only at 48.8s).
          // Blackboard::set is mutex-protected in BT.CPP v3, so writing from
          // this spin thread is safe.
          config().blackboard->set("llm_action", msg->data);
        });
      sub_reason_ = local_node_->create_subscription<std_msgs::msg::String>(
        "llm_reason", rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          llm_reason_ = msg->data;
          config().blackboard->set("llm_reason", msg->data);
        });
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("CheckLlmAction"),
                  "CheckLlmAction: subscribed to %s/llm_action + llm_reason",
                  node->get_namespace());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("CheckLlmAction"),
                  "blackboard has no 'node' — cannot subscribe to llm_action");
    }
  }

  ~CheckLlmAction()
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
    config().blackboard->set("llm_action", llm_action_);
    config().blackboard->set("llm_reason", llm_reason_);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_action_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_reason_;
  rclcpp::Node::SharedPtr local_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;

  std::string llm_action_;
  std::string llm_reason_;
};

}  // namespace social_bt

#endif  // CHECK_LLM_ACTION_HPP_
