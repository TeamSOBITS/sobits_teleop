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
  double lower;
  double upper;
};

struct JointMap {
  std::string joint_group;
  std::string joint_name;
  std::string joint_trajectory_topic;
  int button;
  int fast_button;
  int axis;
  int axis_sign;
  float speed;
  float fast_speed;
  double min_pos;
  double max_pos;
};

struct PoseMap {
  std::string pose_name;
  int trigger;
  int button;
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

struct QuestControllerMap {
  std::string controller_group;
  std::string controller_name;
  std::string arm;
  std::string base_frame_name;
  std::string end_effector_frame_name;
  std::string target_frame_name;
  std::string hand;
  std::string head_joint_trajectory_topic;
  std::string arm_joint_trajectory_topic;
  std::string hand_joint_trajectory_topic;
  std::string type_joint;
  std::vector<std::string> names;
  int arm_mode;
  float scale;
  int gripper_mode;
  int hand_pose_button = -1;    // button to toggle open/close hand pose
  std::string hand_pose_open;   // pose name sent when toggling open
  std::string hand_pose_close;  // pose name sent when toggling close
  std::string hand_pose_action; // action server name for hand pose (e.g. "move_left_hand_to_pose")
  int adaptive_trigger_axis = -1;   // trigger axis that enables adaptive grip
  int adaptive_stick_axis   = -1;   // stick axis that controls open/close
  int adaptive_close_sign   = 1;    // +1: positive stick = close, -1: negative stick = close
  std::vector<AdaptiveJointTarget> adaptive_joints;  // per-joint targets for adaptive grip
  int axis;
  int axis_sign;
  float speed;
  int type_axis = -1;   // single_joint_axis: -1 = feature off
  // Proximity thresholds: arm only latches when controller is within these
  // distances of the robot end-effector. Set to <=0 to disable the check.
  double arm_proximity_threshold = 0.15;       // metres (position)
  double arm_proximity_angle_threshold = 0.52; // radians (~30 deg, orientation)
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
  // Per-controller hand pose action clients, keyed by controller name (e.g. "left", "right").
  // Created at startup from hand_pose_action in each controller's config.
  std::map<std::string,
    rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SharedPtr>
    hand_pose_clients_;

  rclcpp::TimerBase::SharedPtr timer;

  bool joint_state_initialized = false;
  bool joy_received = false;
  bool requires_joint_states = false;
  bool has_quest_controls = false;

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
  std::map<std::string, QuestControllerMap> quest_controller_mappings;
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

  std::vector<std::string> controller_types;

  bool head_control_enabled = false;
  bool arm_control_enabled = false;
  bool gripper_control_enabled = false;

  bool head_tracking = false;
  bool arm_tracking = false;
  bool right_arm_latched = false;
  bool left_arm_latched  = false;

  // Hand pose toggle state per controller: true = open, false = closed
  std::map<std::string, bool>          hand_open_state_;    // keyed by controller name
  std::map<std::string, rclcpp::Time>  hand_toggle_time_;   // debounce timestamp per controller

  tf2::Transform last_tf;
  tf2::Transform current_tf;
  tf2::Transform T_delta;
  double last_pan, last_tilt, last_body_lift;
  double roll, pitch, yaw;
  double dz;
  double pan_target, tilt_target, body_lift_target;
  double target_rad = 0.0;

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
  QuestControllerMap qcm;
  QuestHeadMap qhm;
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__SOBITS_TELEOP_HPP_
