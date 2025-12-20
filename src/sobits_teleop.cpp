#include "sobits_teleop/sobits_teleop.hpp"

SOBITSTeleop::SOBITSTeleop() : Node("sobits_teleop")
{
  std::string robot_name = this->declare_parameter<std::string>("robot_name", "");
  std::string yaml_path = this->declare_parameter<std::string>("mapping_yaml", "");
  std::string joint_states_topic = "/" + robot_name + "/joint_states";

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  move_to_pose_client_ = rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
    this, "/" + robot_name + "/move_to_pose");

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

  if (config["joints"]) {
    auto joints = config["joints"];
    for (auto it : joints) {
      auto joint_name = it.first.as<std::string>();
      auto node = it.second;

      joy_map_.joint = joint_name;
      joy_map_.joint_trajectory_topic = node["joint_trajectory_topic"].as<std::string>();
      joy_map_.mode_button = node["mode_button"].as<int>();
      joy_map_.fast_mode_button = node["fast_mode_button"].as<int>();
      joy_map_.axis = node["axis"].as<int>();
      joy_map_.axis_sign = node["axis_sign"].as<int>();
      joy_map_.speed = node["speed"].as<double>();
      joy_map_.fast_speed = node["fast_speed"].as<double>();
      joy_map_.min_pos = node["min_pos"].as<double>();
      joy_map_.max_pos = node["max_pos"].as<double>();
      joint_mappings_[joint_name] = joy_map_;

      joint_pub_[joy_map_.joint_trajectory_topic] =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          joy_map_.joint_trajectory_topic, 10);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu joint mappings", joint_mappings_.size());
  }
  else RCLCPP_WARN(get_logger(), "joints mapping not found in YAML.");

  if (config["poses"]) {
    auto poses = config["poses"];
    for (auto it : poses) {
      PoseMap pose_map;
      pose_map.pose_name = it.first.as<std::string>();
      pose_map.pose_button = it.second["pose_button"].as<std::vector<int>>();
      pose_mappings_.push_back(pose_map);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose mappings", pose_mappings_.size());
  }
  else RCLCPP_WARN(get_logger(), "poses mapping not found in YAML.");

  if (config["cmd_vel"]) {
    auto c = config["cmd_vel"];
    cmd_vel_map_.cmd_vel_topic = c["cmd_vel_topic"].as<std::string>();
    
    if (c["mode_button"]) cmd_vel_map_.mode_button = c["mode_button"].as<int>();
    if (c["fast_mode_button"]) cmd_vel_map_.fast_mode_button = c["fast_mode_button"].as<int>();
    if (c["mode_axis"]) cmd_vel_map_.mode_axis = c["mode_axis"].as<int>();
    if (c["fast_mode_axis"]) cmd_vel_map_.fast_mode_axis = c["fast_mode_axis"].as<int>();
    
    cmd_vel_map_.linear_x_axis = c["linear_x_axis"].as<int>();
    cmd_vel_map_.linear_y_axis = c["linear_y_axis"].as<int>();
    cmd_vel_map_.angular_axis  = c["angular_axis"].as<int>();
    cmd_vel_map_.axis_sign  = c["axis_sign"].as<int>();
    cmd_vel_map_.linear_scale  = c["linear_scale"].as<double>();
    cmd_vel_map_.angular_scale = c["angular_scale"].as<double>();
    cmd_vel_map_.fast_linear_scale  = c["fast_linear_scale"].as<double>();
    cmd_vel_map_.fast_angular_scale = c["fast_angular_scale"].as<double>();
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_map_.cmd_vel_topic, 10);
    RCLCPP_INFO(get_logger(), "Loaded cmd_vel mapping");
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
  previous_buttons_ = latest_buttons_;
  latest_axes_ = msg->axes;
  latest_buttons_ = msg->buttons;
  joy_received_ = true;
}


void SOBITSTeleop::joy_loop()
{
  if (!joint_state_initialized_) return;
  if (!joy_received_) return;

  std::map<std::string, trajectory_msgs::msg::JointTrajectory> trajs;

  for (auto &[name, m] : joint_mappings_) {

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

  for (const auto &pose_map : pose_mappings_) {
    bool all_buttons_pressed = true;
    bool button_just_pressed = false;
    for (int button_idx : pose_map.pose_button) {
      if (button_idx >= static_cast<int>(latest_buttons_.size())) {
        all_buttons_pressed = false;
        break;
      }
      if (latest_buttons_[button_idx] == 0) {
        all_buttons_pressed = false;
        break;
      }
      if (!previous_buttons_.empty() && 
          button_idx < static_cast<int>(previous_buttons_.size()) &&
          previous_buttons_[button_idx] == 0 && 
          latest_buttons_[button_idx] == 1) {
        button_just_pressed = true;
      }
    }
    if (all_buttons_pressed && button_just_pressed) {
      if (!move_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "MoveToPose action server not available");
        continue;
      }
      auto goal_msg = sobits_interfaces::action::MoveToPose::Goal();
      goal_msg.pose_name = pose_map.pose_name;
      goal_msg.time_allowance.sec = 10;
      goal_msg.time_allowance.nanosec = 0;
      auto send_goal_options = rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SendGoalOptions();
      send_goal_options.result_callback = 
        [this, pose_name = pose_map.pose_name](const auto &result) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(get_logger(), "Pose '%s' succeeded", pose_name.c_str());
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(get_logger(), "Pose '%s' aborted", pose_name.c_str());
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(get_logger(), "Pose '%s' canceled", pose_name.c_str());
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Pose '%s' unknown result", pose_name.c_str());
              break;
          }
        };
      move_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
      RCLCPP_INFO(get_logger(), "Sending pose: %s", pose_map.pose_name.c_str());
    }
  }

  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Twist stop;
  
  bool cmd_vel_enabled = false;
  bool fast_mode = false;
  
  if (cmd_vel_map_.mode_button >= 0 && 
      cmd_vel_map_.mode_button < static_cast<int>(latest_buttons_.size())) {
    cmd_vel_enabled = (latest_buttons_[cmd_vel_map_.mode_button] == 1);
    if (cmd_vel_map_.fast_mode_button >= 0 && 
        cmd_vel_map_.fast_mode_button < static_cast<int>(latest_buttons_.size())) {
      fast_mode = (latest_buttons_[cmd_vel_map_.fast_mode_button] == 1);
    }
  }
  else if (cmd_vel_map_.mode_axis >= 0 && 
           cmd_vel_map_.mode_axis < static_cast<int>(latest_axes_.size())) {
    cmd_vel_enabled = (latest_axes_[cmd_vel_map_.mode_axis] > 0.5);
    if (cmd_vel_map_.fast_mode_axis >= 0 && 
        cmd_vel_map_.fast_mode_axis < static_cast<int>(latest_axes_.size())) {
      fast_mode = (latest_axes_[cmd_vel_map_.fast_mode_axis] > 0.5);
    }
  }
  
  if (cmd_vel_enabled) {
    const double linear_scale = fast_mode ? cmd_vel_map_.fast_linear_scale : cmd_vel_map_.linear_scale;
    const double angular_scale = fast_mode ? cmd_vel_map_.fast_angular_scale : cmd_vel_map_.angular_scale;

    twist.linear.x = latest_axes_[cmd_vel_map_.linear_x_axis] * linear_scale;
    twist.linear.y = latest_axes_[cmd_vel_map_.linear_y_axis] * linear_scale * cmd_vel_map_.axis_sign;
    twist.angular.z = latest_axes_[cmd_vel_map_.angular_axis] * angular_scale * cmd_vel_map_.axis_sign;

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
