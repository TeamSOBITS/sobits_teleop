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

#include <string>
#include <map>
#include <set>
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

struct Limit
{
  double lower = 0.0;
  double upper = 0.0;
};

struct JointMap
{
  std::string joint_group;
  std::string joint_name;
  std::string joint_trajectory_topic;
  // Enable: button and/or enable_axis. Both -1 means always enabled.
  int button = -1;
  int enable_axis = -1;
  int fast_button = -1;
  int fast_axis = -1;
  int axis = -1;
  int axis_sign = 1;
  // Only drive when |axis| exceeds this one, so a diagonal push on a shared
  // stick does not move both of its joints. -1 = no guard.
  int dominant_over = -1;
  float speed = 0.0f;
  float fast_speed = 0.0f;
  double min_pos = 0.0;
  double max_pos = 0.0;
};

// One target group inside a locally-defined pose: joints + positions published
// together on one trajectory topic.
struct PoseJointGroup
{
  std::string joint_trajectory_topic;
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  // Fires this group alone; the pose-level button still fires every group.
  int button = -1;
};

struct PoseMap
{
  std::string pose_name;
  int trigger = -1;   // optional modifier button; -1 = no modifier required
  int button = -1;
  // Non-empty = pose is defined in YAML and published as joint trajectories;
  // empty = fall back to the MoveToPose action (pose resolved server-side).
  std::vector<PoseJointGroup> joint_groups;
  double time_from_start = 3.0;  // seconds to reach the pose
};

// One group's share of a blend: the joints and the two poses' values for them.
struct PoseBlendGroup
{
  std::string joint_trajectory_topic;
  std::vector<std::string> joint_names;
  std::vector<double> from_positions;
  std::vector<double> to_positions;
};

// Sweeps between two poses. The input's sign picks the endpoint and its size
// the speed, so releasing leaves the joints where they are.
struct PoseBlendMap
{
  std::string name;
  std::vector<PoseBlendGroup> groups;
  int enable_axis = -1;     // -1 with no enable_button = always live
  int enable_button = -1;
  int axis = -1;
  int axis_sign = 1;
  int to_button = -1;       // button alternative to the axis
  int from_button = -1;
  double speed = 0.0;       // rad per legacy 50 ms tick at full deflection
};

// Steps through an ordered list of poses, one per button press.
struct PoseCycleMap
{
  std::string name;
  std::vector<std::string> exclude_groups;
  std::vector<std::string> poses;
  int button = -1;
  size_t index = 0;
  rclcpp::Time last_press{0, 0, RCL_ROS_TIME};
};

struct CmdVelMap
{
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

// One joint inside a tracked group: which TF component drives it and how.
struct TrackedJoint
{
  std::string name;
  bool prismatic = false;    // false = rotation
  int component = 0;         // rotation: 0=roll,1=pitch,2=yaw; prismatic: 0=x,1=y,2=z
  double sign = 1.0;
};

// One control_target group identified by a `joints:` map (any group name).
// Latches on its own axis, independent of every other tracked group.
struct QuestTrackedGroup
{
  std::string group;
  std::string joint_trajectory_topic;
  std::string target_frame_name = "hmd_odom";
  int enable_axis = -1;
  double motion_scale = 1.0;
  std::vector<TrackedJoint> joints;

  bool tracking = false;
  bool control_enabled = false;
  bool tf_ok_prev = false;
  tf2::Transform last_tf;
  std::vector<double> latched_positions;   // parallel to joints
};

struct QuestArmMap
{
  std::string group;        // planning group, e.g. "arm_left"
  std::string controller;   // log label only; set from the group name
  // TF the arm follows, and the frame it is re-broadcast as for RViz.
  std::string controller_frame_name;
  std::string controller_echo_frame_name;
  std::string end_effector_frame_name;
  std::string target_frame_name;
  std::string arm_joint_trajectory_topic;
  int enable_axis = -1;
  float motion_scale = 1.0f;
  // Proximity thresholds: arm only latches when controller is within these
  // distances of the robot end-effector. Set to <=0 to disable the check.
  double proximity_threshold = 0.15;       // metres (position)
  double proximity_angle_threshold = 0.52; // radians (~30 deg, orientation)
};

// One per hand_* group (gripper controlled by a hand controller).
struct QuestHandMap
{
  std::string group;        // e.g. "hand_left"
  int pose_button = -1;    // button to toggle open/close hand pose
  std::string pose_open;   // pose name sent when toggling open
  std::string pose_close;  // pose name sent when toggling close
  std::string pose_action; // action server name for hand pose (e.g. "move_left_hand_to_pose")
  // A pose action in flight suppresses this hand's blend; the deadline
  // backstops a lost result.
  bool hand_pose_in_flight = false;
  rclcpp::Time hand_pose_deadline{0, 0, RCL_ROS_TIME};
};

// Per-controller-side arm tracking state (one entry per "left"/"right" etc.).
struct ArmTrackState
{
  // Copied from the arm map so the TF loop never rebuilds frame names.
  std::string controller_frame_name;
  std::string controller_echo_frame_name;
  bool latched = false;
  bool just_latched = false;
  bool have_pub_prev = false;
  bool tf_ok = false;
  tf2::Transform current_tf;       // controller in base_footprint
  tf2::Transform current_tf_odom;  // same (kept for parity with existing math)
  tf2::Transform current_tf_ee;    // EE in base_footprint
  tf2::Transform T_ctrl_latch, T_ee_latch;
  tf2::Transform T_pub_prev;
  rclcpp::Time latch_time{0, 0, RCL_ROS_TIME};
};

class SOBITSTeleop : public rclcpp::Node {
public:
  explicit SOBITSTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Records the key, then forwards to get_parameter.
  template<typename T>
  bool get_param(const std::string & key, T & out)
  {
    read_keys_.insert(key);
    return this->get_parameter(key, out);
  }
  bool has_param(const std::string & key)
  {
    read_keys_.insert(key);
    return this->has_parameter(key);
  }
  // Marks a config subtree as entered even if left empty, so a group the code
  // chose not to populate isn't flagged the same as one never looked at.
  void mark_visited(const std::string & prefix) {visited_prefixes_.insert(prefix);}
  // Warns for unread keys under the known config prefixes; see .cpp for rules.
  void warn_unknown_parameters();

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

  // Send one configured pose. Shared by the joy buttons and the reset service.
  // Publishes the pose, skipping any group named in exclude.
  bool send_pose(
    const PoseMap & pose_map, const std::vector<std::string> & exclude = {},
    const PoseJointGroup * only = nullptr);
  // Publish the tracking enable/disable state for one arm's planning group.
  void publish_arm_tracking(const std::string & arm, bool enabled);
  // True if any arm mapping's own enable axis is currently held.
  bool any_arm_enable_held();
  // Logs which optional config blocks are present/absent at startup.
  void report_config_summary();
  // robot.yaml topic for a group; empty (and reported) if the group has none.
  std::string group_trajectory_topic(const std::string & group);
  // True if the button index is valid and currently down.
  bool button_down(int idx) const;
  // True on the rising edge of a button (down now, up on the previous tick).
  bool button_pressed(int idx) const;
  // True if the axis index is valid and past the hold threshold.
  bool axis_held(int idx) const;
  // Axis value, or 0.0 when the index is out of range.
  double axis_value(int idx) const;
  // Runs the latch/ramp/rate-limit/publish pipeline for one arm controller.
  void process_arm(const QuestArmMap & m, ArmTrackState & st, bool head_tf_ok, bool arm_enabled);
  void process_joints();
  void process_poses();
  void process_pose_blends();
  void process_pose_cycles();
  const PoseMap * find_pose(const std::string & pose);
  // Reverse of group_trajectory_topic, for matching exclude_groups entries.
  std::string topic_group_name(const std::string & topic);
  void process_cmd_vel();
  void process_tracked_group(QuestTrackedGroup & g);
  void process_hand(const std::string & name, QuestHandMap & m);
  // Looks up a Quest frame under base_footprint; rejects stale/invalid TF.
  bool lookup_quest_frame(
    const std::string & quest_frame, tf2::Transform & out,
    tf2::Transform * out_base = nullptr);

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
  // Hand pose action clients, keyed by hand group name (e.g. "hand_left").
  // Created at startup from pose_action in each hand group's config.
  std::map<std::string,
    rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SharedPtr>
  hand_pose_clients_;

  rclcpp::TimerBase::SharedPtr timer;

  bool joint_state_initialized = false;
  bool joy_received = false;
  bool requires_joint_states = false;
  bool has_control_targets = false;
  // Previous tick's cmd_vel enable state, for edge-triggered stop publish.
  bool cmd_vel_was_enabled_ = false;

  std::string robot_description_source_node;
  std::shared_ptr<rclcpp::AsyncParametersClient> async_param_client;
  std::shared_future<std::vector<rclcpp::Parameter>> robot_desc_future;
  rclcpp::TimerBase::SharedPtr urdf_timer;
  bool urdf_loaded = false;
  bool robot_desc_requested = false;

  std::unordered_map<std::string, Limit> joint_limits;

  // Config keys read via get_param/has_param, for warn_unknown_parameters().
  std::set<std::string> read_keys_;
  // Subtree roots entered via mark_visited(), for warn_unknown_parameters().
  std::set<std::string> visited_prefixes_;

  std::string robot_name;
  std::string joint_states_topic;
  std::vector<std::string> joint_groups;
  std::vector<std::string> joint_names;
  std::map<std::string, JointMap> joint_mappings;
  std::map<std::string, QuestArmMap> quest_arm_mappings;
  std::map<std::string, QuestHandMap> quest_hand_mappings;
  std::map<std::string, QuestTrackedGroup> quest_tracked_groups;
  std::map<std::string, double> joint_pos;
  // Frame all Quest tracking resolves in; must be fixed to the arm's root.
  std::string base_frame = "base_footprint";
  double teleop_rate_hz = 100.0;
  double tick_period() const {return 1.0 / teleop_rate_hz;}
  // Trajectory horizon, always two ticks: one tick stalls the controller on any
  // timer jitter, and a horizon past the tick leaves every goal unfinished.
  double dt() const {return 2.0 * tick_period();}
  // Scales legacy per-50ms-tick config speeds to the actual loop period.
  double jog_tick_scale_ = 1.0;

  std::vector<std::string> poses_name;
  std::vector<PoseMap> pose_mappings;
  std::vector<PoseBlendMap> pose_blends;
  std::vector<PoseCycleMap> pose_cycles;

  std::vector<float> latest_axes;
  std::vector<int> latest_buttons;
  std::vector<int> previous_buttons;

  std::vector<std::string> quest_groups;

  bool arm_tracking = false;
  // Per-controller-side arm tracking state, keyed by controller ("left"/"right").
  std::map<std::string, ArmTrackState> arm_track_;

  // Latch timestamps use the soft-start ramp (grip-squeeze jerk suppression).
  // Seconds over which the post-latch delta ramps from 0 to full authority.
  static constexpr double kLatchSoftStartSec = 0.5;
  // Quest frames with stamps further than this from the wall clock are treated as
  // absent (a stale cache once caused a 12.6 cm uncommanded lunge).
  static constexpr double kQuestTfMaxAgeSec = 0.5;
  // Safety net: hard cap on how fast the published arm target may move, applied
  // after all other target math. Bounds the damage of any upstream fault.
  static constexpr double kMaxTargetLinVel = 0.5;  // m/s
  static constexpr double kMaxTargetAngVel = 1.5;  // rad/s

  // Hand pose toggle state per controller: true = open, false = closed
  std::map<std::string, bool> hand_open_state_;             // keyed by hand group name
  std::map<std::string, rclcpp::Time> hand_toggle_time_;    // debounce timestamp per hand group

  // Current controller poses in base_footprint (recomputed every tick)
  tf2::Transform current_tf_hmd;

  // Current controller poses in odom (wall-clock, recomputed every tick)
  tf2::Transform current_tf_hmd_odom;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<rclcpp::Clock> wall_clock_;
  // Single wall-clock buffer for all TFs (Quest wall-clock + robot re-stamped).
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  CmdVelMap cvm;
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__SOBITS_TELEOP_HPP_
