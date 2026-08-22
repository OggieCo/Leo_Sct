// IsRobotClose — custom BT condition node for robot-robot social navigation.
//
// Subscribes to the robot's `robot_close` (Bool), `robot_angle` (Float32),
// `robot_dca` (Float32, predicted min approach distance) and `robot_faster`
// (Bool) topics, published by swarm_basics robot_proximity (namespaced).  On
// every tick it decides whether THIS robot should yield to another robot:
//
//   YIELD RULE (right-hand traffic + speed priority):
//     * head-on  (|angle| <= block_angle_deg)          -> yield (both robots)
//     * side conflict (robot_close && robot_dca <= dca_margin):
//         - yield if the other robot is FASTER than us (don't cut off a fast
//           mover), or
//         - yield if the other robot is on OUR RIGHT (we are "the one on the
//           left" -> yield; right-hand traffic).
//     * side-by-side safe pass (large DCA) -> NO yield.
//
//   Keeps yielding up to yield_max_s (default 6 s), then stops and enters a
//   cooldown (cooldown_s, default 8 s) before it can yield again.
//
//   COMMIT-TO-YIELD: once yielding, it only releases after the conflict has
//   been CONTINUOUSLY clear for release_debounce_s (default 1.5 s), avoiding
//   the flicker/re-yield double-stop when the other robot is still crossing.
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
    robot_close_(false), robot_angle_(0.0), robot_dca_(100.0),
    robot_faster_(false),
    yielding_(false), suppressing_(false),
    yield_started_s_(0.0), suppress_until_s_(0.0), clear_since_s_(0.0),
    block_angle_deg_(20.0), side_cone_deg_(20.0), dca_margin_(0.6),
    yield_max_s_(10.0), cooldown_s_(8.0), release_debounce_s_(1.5),
    hold_cone_deg_(45.0)
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
      sub_dca_ = local_node_->create_subscription<std_msgs::msg::Float32>(
        "robot_dca", rclcpp::QoS(10),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          robot_dca_ = msg->data;
        });
      sub_faster_ = local_node_->create_subscription<std_msgs::msg::Bool>(
        "robot_faster", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          robot_faster_ = msg->data;
        });
      event_pub_ = local_node_->create_publisher<std_msgs::msg::String>(
        "bt_social_event", 10);
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(local_node_);
      spin_thread_ = std::make_shared<std::thread>(
        [this]() { executor_->spin(); });
      RCLCPP_INFO(rclcpp::get_logger("IsRobotClose"),
                  "IsRobotClose: subscribed to %s/robot_close + robot_angle "
                  "+ robot_dca + robot_faster "
                  "(head-on=%.0f deg, side=%.0f deg, dca=%.2f m, "
                  "yield=%.0fs, cooldown=%.0fs)",
                  node->get_namespace(), block_angle_deg_, side_cone_deg_,
                  dca_margin_, yield_max_s_, cooldown_s_);
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

    // YIELD TRIGGER (right-hand traffic + speed priority).
    //   head-on  (|angle| <= block): a rover dead-ahead within close range is
    //              ALWAYS a conflict -> yield regardless of DCA.  (The DCA
    //              predictor is unreliable at slow/coasting speeds and would
    //              report the raw distance > margin, letting a head-on
    //              collision through — observed run_2026-08-22_17-43-36.)
    //   side conflict (DCA <= margin): only a genuine predicted miss is a
    //              conflict; then yield if the other robot is FASTER than us,
    //              or if it is on OUR RIGHT (right-hand traffic).
    //   robot_angle sign: + = LEFT, - = RIGHT.
    const bool head_on =
      robot_close_ &&
      std::abs(robot_angle_) <= block_angle_deg_;
    const bool side_conflict =
      robot_close_ && robot_dca_ <= dca_margin_;
    const bool on_our_right = robot_angle_ < -side_cone_deg_;
    const bool trigger =
      head_on || (side_conflict && (robot_faster_ || on_our_right));

    // HOLD: stay stopped until the other robot has ACTUALLY passed — its
    // bearing moved beyond the hold cone (it is beside/behind us) or it left
    // close range.  Then continue straight with minimal deviation.  The arc
    // is only the fallback when the yield times out without a pass.
    const bool hold =
      robot_close_ &&
      (std::abs(robot_angle_) <= hold_cone_deg_ || on_our_right);

    // Cooldown: let the robot resume once the suppression window elapses.
    if (suppressing_ && now >= suppress_until_s_) {
      suppressing_ = false;
      publish_event("ROBOT_SUPPRESS_END");
      config().blackboard->set("just_yielded", false);  // spin flag consumed
    }

    // Yield decision: trigger satisfied AND not cooling down.
    const bool should_yield = trigger && !suppressing_;

    if (should_yield && !yielding_) {
      yielding_ = true;
      yield_started_s_ = now;
      clear_since_s_ = 0.0;
      publish_event("ROBOT_YIELD_START");
    }

    if (yielding_) {
      // COMMIT TO THE YIELD: hold while the other robot is still in the way
      // (bearing/range based, see `hold` above).  Only release after it has
      // been continuously clear for release_debounce_s_ — prevents both the
      // double-stop and the stop-go limit cycle.
      if (hold) {
        clear_since_s_ = 0.0;                 // still blocked -> reset debounce
      } else if (clear_since_s_ == 0.0) {
        clear_since_s_ = now;                 // conflict gone -> start debounce
      }

      // Give the other robot a fixed window, then stop + cool down.
      // The arc is ONLY the "still blocked after the window" fallback: if the
      // other robot has already passed (bearing beyond the hold cone), resume
      // straight instead — otherwise a very close pass that stays inside
      // close-range the whole window would arc anyway (observed 18-09-02).
      if (now - yield_started_s_ >= yield_max_s_) {
        if (hold) {
          yielding_ = false;
          suppressing_ = true;
          suppress_until_s_ = now + cooldown_s_;
          clear_since_s_ = 0.0;
          publish_event("ROBOT_SUPPRESS_START");
          // Signal the tree: the yield timed out -> run the one-time arc.
          config().blackboard->set("just_yielded", true);
        } else {
          // Other robot already passed -> resume, no arc.
          yielding_ = false;
          clear_since_s_ = 0.0;
          publish_event("ROBOT_YIELD_END");
        }
      } else if (clear_since_s_ > 0.0 &&
                 now - clear_since_s_ >= release_debounce_s_) {
        // Other robot truly gone -> resume without arc.
        yielding_ = false;
        clear_since_s_ = 0.0;
        publish_event("ROBOT_YIELD_END");
      }
    }

    // Feed the tree: robot_close true while we are actually yielding.
    config().blackboard->set("robot_close", yielding_);
    config().blackboard->set("robot_angle", static_cast<double>(robot_angle_));

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
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_dca_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_faster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Node::SharedPtr local_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> spin_thread_;

  bool robot_close_;
  float robot_angle_;
  float robot_dca_;
  bool robot_faster_;
  bool yielding_;
  bool suppressing_;
  double yield_started_s_;
  double suppress_until_s_;
  double clear_since_s_;
  double block_angle_deg_;
  double side_cone_deg_;
  double dca_margin_;
  double yield_max_s_;
  double cooldown_s_;
  double release_debounce_s_;
  double hold_cone_deg_;
};

}  // namespace social_bt

#endif  // IS_ROBOT_CLOSE_HPP_
