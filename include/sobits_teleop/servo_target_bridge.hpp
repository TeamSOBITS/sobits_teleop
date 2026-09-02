#ifndef SOBITS_TELEOP__SERVO_TARGET_BRIDGE_HPP_
#define SOBITS_TELEOP__SERVO_TARGET_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <atomic>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sobits_teleop
{

struct ServoBridgeArmConfig
{
  // Same frame vocabulary as the control_cartesian blocks in quest.yaml:
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
  // Singularity escape: on HALT_FOR_SINGULARITY the EE is parked ON the singular
  // pose, so re-latching there re-traps. Override target with last healthy pose.
  double escape_step{0.0};      // m per tick toward the escape pose; <=0 disables
  double escape_timeout_s{0.0}; // release the override after this long
  // Halt reset: servo ignores pose commands while halted, so the latched state
  // must be cleared with a pause(true)->pause(false) cycle before escaping.
  bool reset_on_halt{true};
  double reset_cooldown_s{1.0}; // min gap between reset attempts
  // Jointspace escape: replay the last healthy joint configuration straight to
  // the arm controller, bypassing servo (which cannot move out of a singularity).
  std::string joint_traj_topic;
  double joint_escape_time_s{0.0};      // trajectory duration; <=0 disables
  double joint_escape_lookback_s{1.0};  // how far back the escape pose is taken from
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

  // Declare key if absent, then read it. Records key into declared_keys_.
  template<typename T>
  T declare_param(const std::string & key, const T & default_value)
  {
    declared_keys_.insert(key);
    if (!this->has_parameter(key)) {
      this->declare_parameter<T>(key, default_value);
    }
    return this->get_parameter(key).get_value<T>();
  }

  // Per-arm key "servo_bridge.<arm>.<key>", defaulting to the shared value.
  template<typename T>
  T declare_arm_param(const std::string & arm, const std::string & key, const T & shared)
  {
    return declare_param("servo_bridge." + arm + "." + key, shared);
  }

  // Warns for any "servo_bridge." override key not recorded in declared_keys_.
  void warn_unknown_parameters();

  // Halt recovery runs as an explicit sequence so every exit path is visible.
  enum class RecoveryState { IDLE, PAUSING, ESCAPING, RESUMING };

  struct ArmBridgeData
  {
    ServoBridgeArmConfig config;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Client<ServoCommandType>::SharedPtr switch_command_type_client;
    rclcpp::Client<SetBool>::SharedPtr pause_servo_client;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub;
    rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub;
    // Servo's own command output — the only place the group's joint names and
    // their live positions appear together.
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_cmd_sub;

    // Slow retry timer used only for the one-time startup sequence
    // (switch_command_type(POSE) + pause_servo(true)); cancelled once both succeed.
    rclcpp::TimerBase::SharedPtr startup_retry_timer;

    std::atomic<bool> enabled{false};
    std::atomic<bool> command_type_set{false};
    std::atomic<bool> initial_pause_set{false};

    // Pause state to reconcile with the servo: -1 none, 0 unpause, 1 pause.
    // A newer toggle overwrites this before the older one's response lands.
    std::atomic<int> pending_pause{-1};

    // ── Singularity escape ──
    // Set in status_callback, read by pose_timer_callback (same executor thread).
    std::atomic<bool> singularity_halt{false};

    // Last healthy EE pose — the escape target.
    tf2::Transform last_good_cmd;
    bool have_last_good{false};

    // Command walked back during an escape; seeded once per halt so the step
    // integrates instead of restarting from the operator target each tick.
    tf2::Transform escape_cmd;
    bool escaping{false};

    // Halt start time; escape gives up after escape_timeout_s so a persistent
    // halt cannot pin the override forever.
    rclcpp::Time halt_start{0, 0, RCL_ROS_TIME};
    bool escape_gave_up{false};

    // Last halt-reset attempt, for the cooldown.
    rclcpp::Time last_reset{0, 0, RCL_ROS_TIME};

    // Halt recovery: PAUSING -> ESCAPING -> RESUMING -> IDLE. escape_done is the
    // deadline (checked by the pose timer) before RESUMING may start.
    RecoveryState recovery{RecoveryState::IDLE};
    rclcpp::Time escape_done{0, 0, RCL_ROS_TIME};

    // Jointspace escape: last healthy joint configuration for this arm's group,
    // replayed to the controller when servo halts.
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub;
    std::vector<std::string> escape_joint_names;
    std::vector<double> escape_joint_positions;
    // Stamped so the window is bounded by elapsed time, not sample count:
    // servo's publish rate is its own config and may not match ours.
    struct JointSample
    {
      rclcpp::Time stamp;
      std::vector<double> positions;
    };
    std::deque<JointSample> joint_history;
    bool have_escape_joints{false};

    // Clears escape/recovery state; shared by disable and other resets.
    void reset_escape_state()
    {
      have_last_good = false;
      escape_gave_up = false;
      escaping = false;
      recovery = RecoveryState::IDLE;
      joint_history.clear();
      have_escape_joints = false;
    }
  };

  void enable_callback(const std::string & arm_name, const std_msgs::msg::Bool::SharedPtr msg);

  // Latches/clears the per-arm singularity halt flag and stamps halt_start.
  void status_callback(
    const std::string & arm_name, const moveit_msgs::msg::ServoStatus::SharedPtr msg);

  // Clears a latched singularity halt with a pause(true)->pause(false) cycle.
  void reset_after_halt(const std::string & arm_name);

  // Advances a PAUSING/ESCAPING/RESUMING recovery once its deadline passes.
  void tick_recovery(const std::string & arm_name);

  // Caches the group's joint names/positions from servo's command stream.
  void servo_cmd_callback(
    const std::string & arm_name,
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  // Publishes the cached healthy joint configuration to the arm controller.
  void send_joint_escape(const std::string & arm_name);

  // Fires per arm on a slow timer until the startup POSE switch and pause
  // both succeed, then cancels itself.
  void try_startup_sequence(const std::string & arm_name);

  // Sends pause_servo(pending_pause) if the service is ready; retried by the
  // startup_retry_timer until the response confirms the latest toggle.
  void try_send_pause(const std::string & arm_name);

  // Shared timer tick: for each enabled arm, TF-lookup + publish PoseStamped.
  void pose_timer_callback();

  std::unordered_map<std::string, std::unique_ptr<ArmBridgeData>> arms_;
  std::unordered_set<std::string> declared_keys_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr pose_timer_;
  double pose_rate_hz_{100.0};
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__SERVO_TARGET_BRIDGE_HPP_
