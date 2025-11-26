#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <map>
#include <vector>
#include <functional>

struct JoyMap {
  std::string joint;
  std::string controller;
  int mode_button;
  int axis;
  float pos_value;
  float neg_value;
  float speed;
  double min_pos;
  double max_pos;
  double smoothing;
};

struct CmdVelMap {
  std::string topic;
  int mode_button;
  int linear_x_axis;
  int linear_y_axis;
  int angular_axis;
  double linear_scale;
  double angular_scale;
};

class JoyTeleop : public rclcpp::Node {
public:
  JoyTeleop();

private:
  void load_mapping(const std::string &yaml_path);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void update_loop();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::map<std::string,
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
    publishers_;

  std::map<std::string, JoyMap> mapping_;
  std::map<std::string, double> joint_pos_;
  std::map<std::string, double> prev_cmd_;

  std::vector<float> latest_axes_;
  std::vector<int> latest_buttons_;
  bool joy_received_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  JoyMap m;
  CmdVelMap cmd_vel_map_;
};
