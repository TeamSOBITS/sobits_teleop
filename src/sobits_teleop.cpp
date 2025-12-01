#include "sobits_teleop/sobits_teleop.hpp"

SOBITSTeleop::SOBITSTeleop() : Node("sobits_teleop")
{
  std::string joint_states_topic = this->declare_parameter<std::string>("joint_states_topic", "");
  std::string yaml_path = this->declare_parameter<std::string>("mapping_yaml", "");

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&SOBITSTeleop::joy_callback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, 10,
    std::bind(&SOBITSTeleop::joint_state_callback, this, std::placeholders::_1));

  if (!yaml_path.empty()) {
    load_mapping(yaml_path);
    RCLCPP_INFO(get_logger(), "Loaded mapping_yaml from: %s", yaml_path.c_str());
  }
  else RCLCPP_ERROR(get_logger(), "mapping_yaml parameter is empty");

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SOBITSTeleop::joy_loop, this));
}

void SOBITSTeleop::load_mapping(const std::string &yaml_path)
{
  YAML::Node config = YAML::LoadFile(yaml_path);

  auto joints = config["joints"];
  for (auto it : joints) {
    auto joint_name = it.first.as<std::string>();
    auto node = it.second;

    m.joint = joint_name;
    m.joint_trajectory_topic = node["joint_trajectory_topic"].as<std::string>();
    m.mode_button = node["mode_button"].as<int>();
    m.fast_mode_button = node["fast_mode_button"].as<int>();
    m.axis = node["axis"].as<int>();
    m.axis_sign = node["axis_sign"].as<int>();
    m.speed = node["speed"].as<double>();
    m.fast_speed = node["fast_speed"].as<double>();
    m.min_pos = node["min_pos"].as<double>();
    m.max_pos = node["max_pos"].as<double>();

    mapping_[joint_name] = m;

    joint_pub_[m.joint_trajectory_topic] =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        m.joint_trajectory_topic, 10);
  }

  if (config["cmd_vel"]) {
    auto c = config["cmd_vel"];
    cmd_vel_map_.cmd_vel_topic = c["cmd_vel_topic"].as<std::string>();
    cmd_vel_map_.mode_button   = c["mode_button"].as<int>();
    cmd_vel_map_.fast_mode_button = c["fast_mode_button"].as<int>();
    cmd_vel_map_.linear_x_axis = c["linear_x_axis"].as<int>();
    cmd_vel_map_.linear_y_axis = c["linear_y_axis"].as<int>();
    cmd_vel_map_.angular_axis  = c["angular_axis"].as<int>();
    cmd_vel_map_.linear_scale  = c["linear_scale"].as<double>();
    cmd_vel_map_.angular_scale = c["angular_scale"].as<double>();
    cmd_vel_map_.fast_linear_scale  = c["fast_linear_scale"].as<double>();
    cmd_vel_map_.fast_angular_scale = c["fast_angular_scale"].as<double>();
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_map_.cmd_vel_topic, 10);
  }
  else RCLCPP_WARN(get_logger(), "cmd_vel mapping not found in YAML. cmd_vel will not be published.");
}

void SOBITSTeleop::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
  joint_pos_[msg->name[i]] = msg->position[i];
  }
  joint_state_initialized_ = true;
}

void SOBITSTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  latest_axes_ = msg->axes;
  latest_buttons_ = msg->buttons;
  joy_received_ = true;
}


void SOBITSTeleop::joy_loop()
{
  if (!joint_state_initialized_) return;
  if (!joy_received_) return;

  std::map<std::string, trajectory_msgs::msg::JointTrajectory> trajs;

  for (auto &[name, m] : mapping_) {

    if (latest_buttons_[m.mode_button] == 0) continue;

    float axis_val = latest_axes_[m.axis];
    if (std::abs(axis_val) < 1e-3) continue;

    double delta_pos = axis_val * m.axis_sign * (latest_buttons_[m.fast_mode_button] == 1 ? m.fast_speed : m.speed);
    joint_pos_[m.joint] += delta_pos;
    joint_pos_[m.joint] = std::clamp(joint_pos_[m.joint], m.min_pos, m.max_pos);

    auto &traj = trajs[m.joint_trajectory_topic];
    traj.joint_names.push_back(m.joint);
    if (traj.points.empty()) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = {joint_pos_[m.joint]};
      p.time_from_start = rclcpp::Duration::from_seconds(dt);
      traj.points.push_back(p);
    }
    else traj.points[0].positions.push_back(joint_pos_[m.joint]);
  }

  for (auto &tj : trajs) {
    const auto &joint_trajectory_topic = tj.first;
    auto &traj = tj.second;
    auto it = joint_pub_.find(joint_trajectory_topic);
    if (it != joint_pub_.end() && traj.joint_names.size() > 0) it->second->publish(traj);
  }

  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Twist stop;
  if (latest_buttons_[cmd_vel_map_.mode_button] == 1) {
    const bool fast = (latest_buttons_[cmd_vel_map_.fast_mode_button] == 1);
    const double linear_scale = fast ? cmd_vel_map_.fast_linear_scale : cmd_vel_map_.linear_scale;
    const double angular_scale = fast ? cmd_vel_map_.fast_angular_scale : cmd_vel_map_.angular_scale;

    twist.linear.x = latest_axes_[cmd_vel_map_.linear_x_axis] * linear_scale;
    twist.linear.y = latest_axes_[cmd_vel_map_.linear_y_axis] * linear_scale;
    twist.angular.z = latest_axes_[cmd_vel_map_.angular_axis] * angular_scale;

    cmd_vel_pub_->publish(twist);
  }
  else cmd_vel_pub_->publish(stop);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SOBITSTeleop>());
  rclcpp::shutdown();
  return 0;
}
