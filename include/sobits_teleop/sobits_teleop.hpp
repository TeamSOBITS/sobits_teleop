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
  int button;
  int fast_button;
  int axis;
  int fast_axis;
  int linear_x_axis;
  int linear_y_axis;
  int angular_axis;
  int axis_sign;
  double linear_scale;
  double angular_scale;
  double fast_linear_scale;
  double fast_angular_scale;
};

class SOBITSTeleop : public rclcpp::Node {
public:
  SOBITSTeleop();

private:
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

  std::string robot_name;
  std::string joint_states_topic;
  std::vector<std::string> joint_groups;
  std::vector<std::string> joint_names;
  std::map<std::string, JointMap> joint_mappings;
  std::map<std::string, double> joint_pos;
  const double dt = 0.1;

  std::vector<std::string> pose_list;
  std::vector<PoseMap> pose_mappings;

  std::vector<float> latest_axes;
  std::vector<int> latest_buttons;
  std::vector<int> previous_buttons;

  JointMap jm;
  PoseMap pm;
  CmdVelMap cvm;
};
