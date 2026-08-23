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
    yielding_(false), suppressing_(false), avoiding_(false),
    yield_started_s_(0.0), suppress_until_s_(0.0), clear_since_s_(0.0),
    avoid_until_s_(0.0),
    block_angle_deg_(20.0), side_cone_deg_(20.0), dca_margin_(0.6),
    yield_max_s_(10.0), cooldown_s_(8.0), release_debounce_s_(1.5),
    hold_cone_deg_(45.0), avoid_window_s_(4.5)
  {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
      local_node_ = std::make_shared<rclcpp::Node>(
        "is_robot_close_node", node->get_namespace(),
        rclcpp::NodeOptions());
      // head-on -> arc-avoidance (both rovers diverge), no tie-break needed
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
      // {robot_avoid} gates the atomic arc branch in both BT trees.
      config().blackboard->set("robot_avoid", false);
      RCLCPP_INFO(rclcpp::get_logger("IsRobotClose"),
                  "IsRobotClose: subscribed to %s/robot_close + robot_angle "
                  "+ robot_dca + robot_faster "
                  "(head-on=%.0f deg -> arc avoid %.1fs window, side=%.0f deg, "
                  "dca=%.2f m, yield=%.0fs, cooldown=%.0fs)",
                  node->get_namespace(), block_angle_deg_, avoid_window_s_,
                  side_cone_deg_, dca_margin_, yield_max_s_, cooldown_s_);
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

    // CLASSIFY (right-hand traffic + speed priority).
    //   head-on  (|angle| <= block): AVOIDANCE, not a stop.  Both rovers run
    //              the atomic arc maneuver (arc "away") — they face OPPOSITE
    //              ways so they DIVERGE and pass; then the 1 Hz replan routes
    //              each back toward its goal.  Guaranteed at the BT layer via
    //              {robot_avoid} (the LLM / reactive layers cannot preempt it).
    //   side conflict (DCA <= margin): a rover CROSSING our path -> genuine
    //              give-way; yield if the other is FASTER than us, or if it is
    //              on OUR RIGHT (right-hand traffic).
    //   robot_angle sign: + = LEFT, - = RIGHT.
    const bool head_on =
      robot_close_ &&
      std::abs(robot_angle_) <= block_angle_deg_;
    const bool side_conflict =
      robot_close_ && robot_dca_ <= dca_margin_;
    const bool on_our_right = robot_angle_ < -side_cone_deg_;
    const bool trigger =
      head_on || (side_conflict && (robot_faster_ || on_our_right));

    // HOLD: for side-conflict yields — stay stopped until the other robot has
    // ACTUALLY passed (bearing beyond the hold cone) or left close range.
    const bool hold =
      robot_close_ &&
      (std::abs(robot_angle_) <= hold_cone_deg_ || on_our_right);

    // HEAD-ON AVOIDANCE (independent state machine): on a fresh head-on, arm
    // the atomic arc branch for avoid_window_s_ (covers the ~3.3 s arc + a
    // little follow), then release so the tree resumes normal navigation, and
    // cool down before any re-arc.
    if (head_on && !suppressing_ && !avoiding_) {
      avoiding_ = true;
      avoid_until_s_ = now + avoid_window_s_;
      publish_event("ROBOT_AVOID_START");
      config().blackboard->set("robot_avoid", true);
      // The ai tree (ai_nav.xml) consumes {robot_avoid} atomically; the
      // reactive tree (social_nav.xml) consumes {just_yielded} for its own
      // ArcOrFollow.  Setting both keeps the shared node tree-agnostic.
      config().blackboard->set("just_yielded", true);
    }
    if (avoiding_ && now >= avoid_until_s_) {
      avoiding_ = false;
      suppressing_ = true;
      suppress_until_s_ = now + cooldown_s_;
      publish_event("ROBOT_AVOID_END");
      config().blackboard->set("robot_avoid", false);
      config().blackboard->set("just_yielded", false);
    }

    // Cooldown: let the robot resume once the suppression window elapses.
    if (suppressing_ && now >= suppress_until_s_) {
      suppressing_ = false;
      publish_event("ROBOT_SUPPRESS_END");
      config().blackboard->set("just_yielded", false);  // spin flag consumed
    }

    // SIDE-CONFLICT YIELD (head-on goes through robot_avoid, not here).
    const bool should_yield = trigger && !suppressing_ && !avoiding_;

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

      // Give the crossing rover the full give-way window, then stop + cool
      // down; the arc is the "still blocked after the window" fallback.  If
      // the other robot has already passed (bearing beyond the hold cone),
      // resume straight instead.
      if (now - yield_started_s_ >= yield_max_s_) {
        if (hold) {
          yielding_ = false;
          suppressing_ = true;
          suppress_until_s_ = now + cooldown_s_;
          clear_since_s_ = 0.0;
          publish_event("ROBOT_SUPPRESS_START");
          // Signal the tree: run the one-time arc, then FollowPath replans.
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
  bool avoiding_;                 // head-on avoidance maneuver in progress
  double yield_started_s_;
  double suppress_until_s_;
  double clear_since_s_;
  double avoid_until_s_;
  double block_angle_deg_;
  double side_cone_deg_;
  double dca_margin_;
  double yield_max_s_;
  double cooldown_s_;
  double release_debounce_s_;
  double hold_cone_deg_;
  double avoid_window_s_;         // s the head-on arc branch stays active
};

}  // namespace social_bt

#endif  // IS_ROBOT_CLOSE_HPP_
