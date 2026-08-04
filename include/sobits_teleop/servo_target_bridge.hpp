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

struct ServoBridgeArmConfig
{
  // Same frame vocabulary as the quest_control blocks in quest.yaml:
  // target_frame_name / base_frame_name / end_effector_frame_name.
  std::string target_frame;
  std::string base_frame;
  std::string servo_node;
  std::string enable_topic;
  // end_effector_frame follows the target; servo_ee_frame is what servo drives
  // (empty = same link). When they differ the target is re-expressed each tick.
  std::string end_effector_frame;
  std::string servo_ee_frame;
  // Reach clamp: targets clamped to a max_reach sphere around reach_origin_frame —
  // full extension latches a servo e-stop. <= 0 disables.
  std::string reach_origin_frame;
  double max_reach{0.0};
};

// ServoTargetBridge: streams TF targets into servo's pose_target_cmds per arm,
// gated by */moveit_track_enabled. Never blocks the executor — all calls async.
class ServoTargetBridge : public rclcpp::Node
{
public:
  explicit ServoTargetBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ServoCommandType = moveit_msgs::srv::ServoCommandType;
  using SetBool = std_srvs::srv::SetBool;

  struct ArmBridgeData
  {
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

    // Pause state to reconcile with the servo: -1 none, 0 unpause, 1 pause.
    // A newer toggle overwrites this before the older one's response lands.
    std::atomic<int> pending_pause{-1};
  };

  void enable_callback(const std::string & arm_name, const std_msgs::msg::Bool::SharedPtr msg);

  // Fires per arm on a slow timer until the startup POSE switch and pause
  // both succeed, then cancels itself.
  void try_startup_sequence(const std::string & arm_name);

  // Sends pause_servo(pending_pause) if the service is ready; retried by the
  // startup_retry_timer until the response confirms the latest toggle.
  void try_send_pause(const std::string & arm_name);

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
