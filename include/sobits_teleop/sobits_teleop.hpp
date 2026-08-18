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
  int button = -1;
  int fast_button = -1;
  int axis = -1;
  int axis_sign = 1;
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

// One quest_control group identified by a `joints:` map (any group name).
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

// Per-joint adaptive gripper target (close and open positions).
// Joints omitted from this list are not commanded by the adaptive gripper.
struct AdaptiveJointTarget
{
  std::string name;
  float close_pos;
  float open_pos;
  bool fixed = false;  // true: always commanded at close_pos regardless of direction
};

// One per arm_* group (planning group tracked by an arm controller).
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
  float speed = 0.2f;
  int adaptive_trigger_axis = -1;   // trigger axis that enables adaptive grip
  int adaptive_stick_axis = -1;     // stick axis that controls open/close
  int adaptive_close_sign = 1;      // +1: positive stick = close, -1: negative stick = close
  std::vector<AdaptiveJointTarget> adaptive_joints;  // per-joint targets for adaptive grip
  std::string type_joint;
  int type_axis = -1;   // single_joint.axis: -1 = feature off
  int type_sign = 1;    // single_joint.axis_sign: +1/-1 flips the vertical-jog direction
  // Functional range for the vjog joint (single_joint.min/max) — the
  // switching-gear mechanism can jam outside it.
  float type_min = -1e9f;
  float type_max = 1e9f;
  // Endpoint-swing state: one swing per stick flick, re-armed at stick center;
  // the endpoint is re-commanded every goal until arrival (beats the gear spring).
  bool vjog_armed = true;
  bool vjog_swing_active = false;
  float vjog_swing_target = 0.0f;
  rclcpp::Time vjog_swing_deadline{0, 0, RCL_ROS_TIME};
  // One adaptive MoveJoint goal in flight per hand; deadline backstops a lost result.
  bool jog_goal_in_flight = false;
  rclcpp::Time jog_goal_deadline{0, 0, RCL_ROS_TIME};
  // Adaptive stream pauses while a pose action runs, else per-tick "hold" goals
  // override the pose trajectory; deadline backstops a lost result.
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
  bool send_pose(const PoseMap & pose_map);
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

  std::vector<std::string> pose_list;
  std::vector<PoseMap> pose_mappings;

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
