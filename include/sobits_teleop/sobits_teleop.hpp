#ifndef SOBITS_TELEOP__SOBITS_TELEOP_HPP_
#define SOBITS_TELEOP__SOBITS_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "sobits_interfaces/action/move_to_pose.hpp"
#include "sobits_interfaces/action/move_joint.hpp"

#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <urdf/model.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sobits_teleop
{

struct Limit {
  double lower = 0.0;
  double upper = 0.0;
};

struct JointMap {
  std::string joint_group;
  std::string joint_name;
  std::string joint_trajectory_topic;
  int button = -1;
  int fast_button = -1;
  int axis = -1;
  int axis_sign = 1;
  float speed = 0.0f;
  float fast_speed = 0.0f;
  double min_pos = 0.0;
  double max_pos = 0.0;
};

struct PoseMap {
  std::string pose_name;
  int trigger = -1;   // optional modifier button; -1 = no modifier required
  int button = -1;
};

struct CmdVelMap {
  std::string topic;
  int button = -1;
  int fast_button = -1;
  int axis = -1;
  int fast_axis = -1;
  int linear_x_axis = -1;
  int linear_y_axis = -1;
  int angular_axis = -1;
  int axis_sign = 1;
  double linear_scale = 0.0;
  double angular_scale = 0.0;
  double fast_linear_scale = 0.0;
  double fast_angular_scale = 0.0;
};

struct QuestHeadMap {

  std::string head_joint_trajectory_topic;
  std::string body_joint_trajectory_topic;
  std::string vertical;
  std::string horizontal;
  std::string body_lift;
  int head_mode;
  int vertical_sign;
  int horizontal_sign;
  float scale;
};

// Per-joint adaptive gripper target (close and open positions).
// Joints omitted from this list are not commanded by the adaptive gripper.
struct AdaptiveJointTarget {
  std::string name;
  float close_pos;
  float open_pos;
  bool fixed = false;  // true: always commanded at close_pos regardless of direction
};

// One per arm_* group (planning group tracked by an arm controller).
struct QuestArmMap {
  std::string group;        // planning group, e.g. "arm_left"
  std::string controller;   // "left" or "right"
  std::string base_frame_name;
  std::string end_effector_frame_name;
  std::string target_frame_name;
  std::string arm_joint_trajectory_topic;
  int enable_axis = -1;
  float scale = 1.0f;
  // Proximity thresholds: arm only latches when controller is within these
  // distances of the robot end-effector. Set to <=0 to disable the check.
  double proximity_threshold = 0.15;       // metres (position)
  double proximity_angle_threshold = 0.52; // radians (~30 deg, orientation)
};

// One per hand_* group (gripper controlled by a hand controller).
struct QuestHandMap {
  std::string group;        // e.g. "hand_left"
  std::string controller;   // "left" or "right"
  int pose_button = -1;    // button to toggle open/close hand pose
  std::string pose_open;   // pose name sent when toggling open
  std::string pose_close;  // pose name sent when toggling close
  std::string pose_action; // action server name for hand pose (e.g. "move_left_hand_to_pose")
  float speed = 0.2f;
  int adaptive_trigger_axis = -1;   // trigger axis that enables adaptive grip
  int adaptive_stick_axis   = -1;   // stick axis that controls open/close
  int adaptive_close_sign   = 1;    // +1: positive stick = close, -1: negative stick = close
  std::vector<AdaptiveJointTarget> adaptive_joints;  // per-joint targets for adaptive grip
  std::string type_joint;
  int type_axis = -1;   // single_joint.axis: -1 = feature off
  int type_sign = 1;    // single_joint.axis_sign: +1/-1 flips the vertical-jog direction
  // Optional functional range for the vjog joint (single_joint.min/max). The jog
  // is confined to this range instead of the full URDF limits — outside it the
  // switching-gear mechanism can jam. If the joint starts outside the range the
  // jog may still step back toward it, never further away.
  float type_min = -1e9f;
  float type_max =  1e9f;
  // Flick re-arm latch for the endpoint-swing jog mode (used when a functional
  // range is configured): one swing per stick flick, re-armed at stick center.
  bool vjog_armed = true;
  // Persistent swing: after a flick, every adaptive goal keeps commanding the
  // knuckle toward the endpoint until it arrives (or the deadline expires).
  // The sustained large position error is what beats the gear spring; carrying
  // it inside ordinary loop-rate goals lets open/close run concurrently
  // instead of being blocked behind one long-running swing goal.
  bool vjog_swing_active = false;
  float vjog_swing_target = 0.0f;
  rclcpp::Time vjog_swing_deadline{0, 0, RCL_ROS_TIME};
  // Per-hand goal gate: one adaptive MoveJoint goal in flight per hand, so the
  // two hands never block each other (the server runs goals on detached
  // threads without preemption; see the send-site comment).
  bool jog_goal_in_flight = false;
  // Deadline backstop for a lost MoveJoint result (e.g. server restart).
  rclcpp::Time jog_goal_deadline{0, 0, RCL_ROS_TIME};
  // While a full-hand pose action (open/close toggle) runs, this hand's
  // adaptive goal stream pauses — otherwise the loop-rate adaptive goals
  // command "hold current" every tick and override the pose trajectory on the
  // same controller, making the button appear dead whenever adaptive mode is
  // engaged. The deadline is a backstop against a lost action result.
  bool hand_pose_in_flight = false;
  rclcpp::Time hand_pose_deadline{0, 0, RCL_ROS_TIME};
};

class SOBITSTeleop : public rclcpp::Node {
public:
  explicit SOBITSTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool parse_urdf_limits(const std::string & urdf_xml);
  void load_joint_limits();
  void load_parameters();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void robot_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void robot_tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void teleop();
  // Clamp value into joint's URDF limits; false (no clamp) if limits unknown.
  bool clamp_to_limits_checked(const std::string & joint, double & value);
  // Publish the tracking enable/disable state for one arm's planning group.
  void publish_arm_tracking(const std::string & arm, bool enabled);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_tf_sub;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_tf_static_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  std::map<std::string,
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
    joint_pub;
  // Publishers to enable/disable MoveIt arm tracking per planning group
  // Key = planning_group name (e.g. "arm_left"), topic = <key>/moveit_track_enabled
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr>
    arm_track_pubs_;

  rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SharedPtr move_to_pose_client;
  rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SharedPtr move_joint_client;
  // Hand pose action clients, keyed by hand group name (e.g. "hand_left").
  // Created at startup from pose_action in each hand group's config.
  std::map<std::string,
    rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SharedPtr>
    hand_pose_clients_;

  rclcpp::TimerBase::SharedPtr timer;

  bool joint_state_initialized = false;
  bool joy_received = false;
  bool requires_joint_states = false;
  bool has_quest_controls = false;
  // Previous tick's cmd_vel enable state, for edge-triggered stop publish.
  bool cmd_vel_was_enabled_ = false;

  std::string robot_description_source_node;
  std::shared_ptr<rclcpp::AsyncParametersClient> async_param_client;
  std::shared_future<std::vector<rclcpp::Parameter>> robot_desc_future;
  rclcpp::TimerBase::SharedPtr urdf_timer;
  bool urdf_loaded = false;
  bool robot_desc_requested = false;

  std::unordered_map<std::string, Limit> joint_limits;

  std::string robot_name;
  std::string joint_states_topic;
  std::vector<std::string> joint_groups;
  std::vector<std::string> joint_names;
  std::map<std::string, JointMap> joint_mappings;
  std::map<std::string, QuestArmMap> quest_arm_mappings;
  std::map<std::string, QuestHandMap> quest_hand_mappings;
  std::map<std::string, double> joint_pos;
  const double dt = 0.1;
  double teleop_rate_hz = 100.0;
  // Config speed values are defined as position delta per legacy 50 ms tick
  // (the timer period when they were tuned). This scales those per-tick deltas
  // to the actual loop period so jog speed is independent of teleop_rate_hz.
  double jog_tick_scale_ = 1.0;

  std::vector<std::string> pose_list;
  std::vector<PoseMap> pose_mappings;

  std::vector<float> latest_axes;
  std::vector<int> latest_buttons;
  std::vector<int> previous_buttons;

  std::vector<std::string> quest_groups;

  bool head_control_enabled = false;
  bool arm_control_enabled = false;

  bool head_tracking = false;
  bool arm_tracking = false;
  bool right_arm_latched = false;
  bool left_arm_latched  = false;
  // Set for exactly one teleop tick when an arm latches: defers the tracking
  // enable publish until after the first re-zeroed target is broadcast on TF.
  bool right_arm_just_latched = false;
  bool left_arm_just_latched  = false;

  // Clutch re-zeroing: controller pose and EE pose captured at the latch
  // instant; while latched the published target is EE_latch composed with the
  // CONTROLLER's delta since latch (scaled), so tracking starts with zero error.
  // Deltas deliberately exclude the HMD: the raw mapping scale*ctrl+(1-scale)*hmd
  // leaks operator head sway into the target whenever scale != 1.
  tf2::Transform T_ctrl_latch_r, T_ee_latch_r;
  tf2::Transform T_ctrl_latch_l, T_ee_latch_l;
  // Latch timestamps for the soft-start ramp (grip-squeeze jerk suppression).
  rclcpp::Time latch_time_r_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latch_time_l_{0, 0, RCL_ROS_TIME};
  // Seconds over which the post-latch delta ramps from 0 to full authority.
  static constexpr double kLatchSoftStartSec = 0.5;
  // Quest TF frames whose stamp differs from the wall clock by more than this
  // are treated as absent. Serving "latest" from the buffer regardless of age
  // let a stale cache (headset disconnect) or future-stamped data (clock-skewed
  // source) shadow live input — the cause of a 12.6 cm uncommanded lunge.
  static constexpr double kQuestTfMaxAgeSec = 0.5;
  // Safety net: hard cap on how fast the published arm target may move, applied
  // after all other target math. Bounds the damage of any upstream fault.
  static constexpr double kMaxTargetLinVel = 0.5;  // m/s
  static constexpr double kMaxTargetAngVel = 1.5;  // rad/s
  tf2::Transform T_pub_prev_r_, T_pub_prev_l_;
  bool have_pub_prev_r_ = false;
  bool have_pub_prev_l_ = false;

  // Hand pose toggle state per controller: true = open, false = closed
  std::map<std::string, bool>          hand_open_state_;    // keyed by hand group name
  std::map<std::string, rclcpp::Time>  hand_toggle_time_;   // debounce timestamp per hand group

  tf2::Transform last_tf;
  tf2::Transform current_tf;
  tf2::Transform T_delta;
  double last_pan, last_tilt, last_body_lift;
  double roll, pitch, yaw;
  double dz;
  double pan_target, tilt_target, body_lift_target;

  // Current controller poses in base_footprint (recomputed every tick)
  tf2::Transform current_tf_hmd;
  tf2::Transform current_tf_r;
  tf2::Transform current_tf_l;
  tf2::Transform current_tf_ee_r;
  tf2::Transform current_tf_ee_l;

  // Current controller poses in odom (wall-clock, recomputed every tick)
  tf2::Transform current_tf_r_odom;
  tf2::Transform current_tf_l_odom;
  tf2::Transform current_tf_hmd_odom;

  tf2::Transform T_target_r;
  tf2::Transform T_target_l;

  geometry_msgs::msg::TransformStamped tf_msg;
  geometry_msgs::msg::TransformStamped target_msg_r;
  geometry_msgs::msg::TransformStamped target_msg_l;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<rclcpp::Clock> wall_clock_;
  // Single wall-clock buffer for all TFs (Quest wall-clock + robot re-stamped).
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  Limit lim;
  JointMap jm;
  PoseMap pm;
  CmdVelMap cvm;
  QuestHeadMap qhm;
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__SOBITS_TELEOP_HPP_
