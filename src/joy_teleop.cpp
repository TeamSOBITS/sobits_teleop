#include "sobits_teleop/joy_teleop.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>

JoyTeleop::JoyTeleop() : Node("joy_teleop")
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&JoyTeleop::joy_callback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&JoyTeleop::joint_state_callback, this, std::placeholders::_1));

  std::string yaml_path = this->declare_parameter<std::string>("mapping_yaml", "");

  if (!yaml_path.empty()) {
    load_mapping(yaml_path);
    RCLCPP_INFO(get_logger(), "Loaded %ld joints", mapping_.size());
  } else {
    RCLCPP_ERROR(get_logger(), "mapping_yaml parameter is empty");
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&JoyTeleop::update_loop, this));
}

void JoyTeleop::load_mapping(const std::string &yaml_path)
{
  YAML::Node config = YAML::LoadFile(yaml_path);

  auto joints = config["joints"];
  for (auto it : joints) {
    auto joint_name = it.first.as<std::string>();
    auto node = it.second;

    m.joint = joint_name;
    m.controller = node["controller"].as<std::string>();
    m.mode_button = node["mode_button"].as<int>();
    m.axis = node["axis"].as<int>();
    m.pos_value = node["pos_value"].as<double>();
    m.neg_value = node["neg_value"].as<double>();
    m.speed = node["speed"].as<double>();
    m.min_pos = node["min_pos"].as<double>();
    m.max_pos = node["max_pos"].as<double>();
    m.smoothing = node["smoothing"].as<double>();

    mapping_[joint_name] = m;

    publishers_[m.controller] =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        m.controller, 10);
  }

  if (config["cmd_vel"]) {
    auto c = config["cmd_vel"];
    cmd_vel_map_.topic         = c["topic"].as<std::string>();
    cmd_vel_map_.mode_button   = c["mode_button"].as<int>();
    cmd_vel_map_.linear_x_axis = c["linear_x_axis"].as<int>();
    cmd_vel_map_.linear_y_axis = c["linear_y_axis"].as<int>();
    cmd_vel_map_.angular_axis  = c["angular_axis"].as<int>();
    cmd_vel_map_.linear_scale  = c["linear_scale"].as<double>();
    cmd_vel_map_.angular_scale = c["angular_scale"].as<double>();
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_map_.topic, 10);
  } else {
    RCLCPP_WARN(get_logger(),
      "cmd_vel mapping not found in YAML. cmd_vel will not be published.");
  }
}

void JoyTeleop::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
  joint_pos_[msg->name[i]] = msg->position[i];
  }
}

void JoyTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  latest_axes_ = msg->axes;
  latest_buttons_ = msg->buttons;
  joy_received_ = true;
}


void JoyTeleop::update_loop()
{
  if (!joy_received_) return;

  for (auto &[name, m] : mapping_) {

    if (m.mode_button < 0 || static_cast<size_t>(m.mode_button) >= latest_buttons_.size()) continue;
    if (latest_buttons_[m.mode_button] == 0) continue;

    if (m.axis < 0 || static_cast<size_t>(m.axis) >= latest_axes_.size()) continue;
    float axis_val = latest_axes_[m.axis];
    double raw_delta = 0.0;

    if (axis_val == m.pos_value)
      raw_delta = m.speed;
    else if (axis_val == m.neg_value)
      raw_delta = -m.speed;
    else
      continue;

    double filtered_delta = raw_delta;
    if (prev_cmd_.count(m.joint))
      filtered_delta = m.smoothing * raw_delta
               + (1 - m.smoothing) * prev_cmd_[m.joint];

    prev_cmd_[m.joint] = filtered_delta;

    joint_pos_[m.joint] += filtered_delta;
    joint_pos_[m.joint] =
      std::clamp(joint_pos_[m.joint], m.min_pos, m.max_pos);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {m.joint};

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {joint_pos_[m.joint]};
    p.time_from_start = rclcpp::Duration::from_seconds(0.1);

    traj.points.push_back(p);
    publishers_[m.controller]->publish(traj);
  }

  bool mode_on = false;
  if (cmd_vel_map_.mode_button >= 0 && static_cast<size_t>(cmd_vel_map_.mode_button) < latest_buttons_.size()) {
    mode_on = (latest_buttons_[cmd_vel_map_.mode_button] == 1);
  }

  int max_axis = std::max({cmd_vel_map_.linear_x_axis,
                           cmd_vel_map_.linear_y_axis,
                           cmd_vel_map_.angular_axis});
  bool axes_ok = (max_axis >= 0) && (static_cast<size_t>(max_axis) < latest_axes_.size());
  geometry_msgs::msg::Twist twist;
  if (mode_on && axes_ok) {
    twist.linear.x =
      latest_axes_[cmd_vel_map_.linear_x_axis] * cmd_vel_map_.linear_scale;

    twist.linear.y =
      latest_axes_[cmd_vel_map_.linear_y_axis] * cmd_vel_map_.linear_scale;

    twist.angular.z =
      latest_axes_[cmd_vel_map_.angular_axis] * cmd_vel_map_.angular_scale;
    cmd_vel_pub_->publish(twist);
  }
  else
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
