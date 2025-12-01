#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

struct CmdVelMap {
  std::string cmd_vel_topic;
  int mode_button;
  int fast_mode_button;
  int linear_x_axis;
  int linear_y_axis;
  int angular_axis;
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

  std::map<std::string, JoyMap> mapping_;
  std::map<std::string, double> joint_pos_;

  std::vector<float> latest_axes_;
  std::vector<int> latest_buttons_;
  bool joint_state_initialized_ = false;
  bool joy_received_ = false;
  const double dt = 0.1;

  rclcpp::TimerBase::SharedPtr timer_;
  JoyMap m;
  CmdVelMap cmd_vel_map_;
};
