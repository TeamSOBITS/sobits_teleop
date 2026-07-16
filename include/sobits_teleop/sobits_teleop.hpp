#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sobits_interfaces/action/move_to_pose.hpp"

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
  double max_lead;      // per-joint override; defaults to control_joints.max_lead
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
  // -1 = disabled, so an omitted control_velocity block can't index out of bounds.
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
  int axis;
  int axis_sign;
  float speed;
  int type_axis;
};

class SOBITSTeleop : public rclcpp::Node {
public:
  SOBITSTeleop();

private:
  bool parse_urdf_limits(const std::string & urdf_xml);
  void load_joint_limits();
  void load_parameters();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void teleop();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  std::map<std::string,
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
    joint_pub;

  rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SharedPtr move_to_pose_client;

  rclcpp::TimerBase::SharedPtr timer;

  bool joint_state_initialized = false;
  bool joy_received = false;

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
  std::map<std::string, double> joint_pos;   // measured, from /joint_states
  std::map<std::string, double> cmd_pos;     // integrated command target
  std::map<std::string, bool> joint_active_prev;  // was the joint driven last tick
  const double dt = 0.1;

  // Set once the matching param block is present; gates the control code below.
  bool cmd_vel_loaded = false;
  bool quest_loaded = false;

  // Overridable via control_joints.command_duration / .max_lead.
  double joint_cmd_duration = 0.08;  // time_from_start per trajectory point (s)
  double joint_max_lead = 0.15;      // max command lead over the real joint (rad)

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

  tf2::Transform last_tf;
  tf2::Transform current_tf;
  tf2::Transform T_delta;
  double last_pan, last_tilt, last_body_lift;
  double roll, pitch, yaw;
  double dz;
  double pan_target, tilt_target, body_lift_target;
  double target_rad = 0.0;

  tf2::Transform last_tf_r;
  tf2::Transform current_tf_r;
  tf2::Transform last_tf_l;
  tf2::Transform current_tf_l;
  tf2::Transform current_tf_ee_r;
  tf2::Transform last_tf_ee_r;
  tf2::Transform current_tf_ee_l;
  tf2::Transform last_tf_ee_l;
  tf2::Transform T_delta_l;
  tf2::Transform T_target_l;
  tf2::Transform T_delta_r;
  tf2::Transform T_target_r;
  tf2::Transform T_delta_r_align;
  tf2::Transform T_delta_l_align;

  tf2::Quaternion q_align;
  tf2::Transform T_align;

  geometry_msgs::msg::TransformStamped tf_msg;
  geometry_msgs::msg::TransformStamped target_msg_r;
  geometry_msgs::msg::TransformStamped target_msg_l;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  Limit lim;
  JointMap jm;
  PoseMap pm;
  CmdVelMap cvm;
  QuestControllerMap qcm;
  QuestHeadMap qhm;
};
