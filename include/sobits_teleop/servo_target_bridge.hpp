#ifndef SOBITS_TELEOP__SERVO_TARGET_BRIDGE_HPP_
#define SOBITS_TELEOP__SERVO_TARGET_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace sobits_teleop
{

struct ServoBridgeArmConfig {
  // Same frame vocabulary as the quest_control blocks in quest.yaml:
  // target_frame_name / base_frame_name / end_effector_frame_name.
  std::string target_frame;
  std::string base_frame;
  std::string servo_node;
  std::string enable_topic;
  // end_effector_frame is the robot link that should FOLLOW the target
  // (matches quest.yaml's end_effector_frame_name). servo_ee_frame is the link
  // servo actually drives (the planning group's last SRDF link); EMPTY means
  // "servo drives end_effector_frame itself" — no correction. When they
  // differ, the bridge re-expresses the target for the servo frame each tick:
  //   T(base->cmd) = T(base->target) * T(end_effector_frame->servo_ee_frame)
  std::string end_effector_frame;
  std::string servo_ee_frame;
  // Reach clamp: targets are clamped to a sphere of radius max_reach around
  // reach_origin_frame before being sent to servo. Chasing an out-of-reach hand
  // target otherwise drives the arm to full extension — a true elbow
  // singularity where servo 2.12 latches an emergency stop that only an
  // external joint-space move can clear. max_reach <= 0 disables the clamp.
  std::string reach_origin_frame;
  double max_reach{0.0};
};

// ---------------------------------------------------------------------------
// ServoTargetBridge
//
// Streams geometry_msgs/PoseStamped targets (read from TF) into a
// moveit_servo::ServoNode's ~/pose_target_cmds topic, one instance per arm,
// gated by the sobits_teleop-published */moveit_track_enabled Bool topic.
//
// This node must NEVER block its executor: all service calls (switch_command_type,
// pause_servo) are async with response callbacks, and the shared timer only does
// a TF lookup + publish per enabled arm (finding 10 in the old moveit_arm_controller
// design was a blocking-call stall — this node is deliberately structured to avoid
// that class of bug).
// ---------------------------------------------------------------------------
class ServoTargetBridge : public rclcpp::Node
{
public:
  explicit ServoTargetBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ServoCommandType = moveit_msgs::srv::ServoCommandType;
  using SetBool = std_srvs::srv::SetBool;

  struct ArmBridgeData {
    ServoBridgeArmConfig config;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Client<ServoCommandType>::SharedPtr switch_command_type_client;
    rclcpp::Client<SetBool>::SharedPtr pause_servo_client;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub;

    // Slow retry timer used only for the one-time startup sequence
    // (switch_command_type(POSE) + pause_servo(true)); cancelled once both succeed.
    rclcpp::TimerBase::SharedPtr startup_retry_timer;

    std::atomic<bool> enabled{false};
    std::atomic<bool> command_type_set{false};
    std::atomic<bool> initial_pause_set{false};
  };

  void enable_callback(const std::string & arm_name, const std_msgs::msg::Bool::SharedPtr msg);

  // Async, non-blocking: fires repeatedly per arm on a slow timer until both the
  // startup POSE command-type switch and the startup pause(true) succeed, then
  // cancels itself.
  void try_startup_sequence(const std::string & arm_name);

  // Shared timer tick: for each enabled arm, TF-lookup + publish PoseStamped.
  void pose_timer_callback();

  std::unordered_map<std::string, std::unique_ptr<ArmBridgeData>> arms_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr pose_timer_;
  double pose_rate_hz_{100.0};
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__SERVO_TARGET_BRIDGE_HPP_
