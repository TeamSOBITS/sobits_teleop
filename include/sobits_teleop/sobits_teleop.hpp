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
#include <yaml-cpp/yaml.h>


struct JoyMap {
  std::string joint;
  std::string joint_trajectory_topic;
  int mode_button;
  int fast_mode_button;
  int axis;
  int axis_sign;
  float speed;
  float fast_speed;
  double min_pos;
  double max_pos;
};

struct PoseMap {
  std::string pose_name;
  std::vector<int> pose_button;
};

struct CmdVelMap {
  std::string cmd_vel_topic;
  int mode_button;
  int mode_axis;
  int fast_mode_button;
  int fast_mode_axis;
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
  void load_mapping(const std::string &yaml_path);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void joy_loop();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::map<std::string,
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
    joint_pub_;

  rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SharedPtr move_to_pose_client_;

  std::map<std::string, JoyMap> joint_mappings_;
  std::map<std::string, double> joint_pos_;
  std::vector<PoseMap> pose_mappings_;

  std::vector<float> latest_axes_;
  std::vector<int> latest_buttons_;
  std::vector<int> previous_buttons_;
  bool joint_state_initialized_ = false;
  bool joy_received_ = false;
  const double dt = 0.1;

  rclcpp::TimerBase::SharedPtr timer_;
  JoyMap joy_map_;
  CmdVelMap cmd_vel_map_;
};
