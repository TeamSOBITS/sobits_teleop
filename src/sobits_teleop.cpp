#include "sobits_teleop/sobits_teleop.hpp"

SOBITSTeleop::SOBITSTeleop()
  : Node(
      "sobits_teleop",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
{
  this->get_parameter("robot_name", robot_name);

  move_to_pose_client = rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
    this, "/" + robot_name + "/move_to_pose");

  joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&SOBITSTeleop::joy_callback, this, std::placeholders::_1));

  load_parameters();

  timer = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SOBITSTeleop::teleop, this));
}

void SOBITSTeleop::load_parameters()
{
  this->get_parameter("robot_topic_name.joint_states_topic", joint_states_topic);
  joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, 10,
    std::bind(&SOBITSTeleop::joint_state_callback, this, std::placeholders::_1));

  this->get_parameter("robot_topic_name.cmd_vel_topic", cvm.topic);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    cvm.topic, 10);

  // Load joint parameters
  // if (this->has_parameter("control_joints")) {
    this->get_parameter("control_joints.groups", joint_groups);

    for (const auto& joint_group : joint_groups) {
      if (!this->get_parameter("control_joints." + joint_group + ".names", joint_names)) continue;
      
      for (const auto& joint_name : joint_names) {
        // if (!this->has_parameter("control_joints." + joint_group + "." + joint_name)) continue;

        jm.joint_group = joint_group;
        jm.joint_name  = joint_name;
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".button",      jm.button);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".fast_button", jm.fast_button);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".axis",        jm.axis);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".axis_sign",   jm.axis_sign);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".speed",       jm.speed);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".fast_speed",  jm.fast_speed);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".min_pos",     jm.min_pos);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".max_pos",     jm.max_pos);
        this->get_parameter("robot_topic_name.joint_trajectory_topic." + joint_group,            jm.joint_trajectory_topic);

        joint_mappings[joint_name] = jm;
        joint_pub[jm.joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          jm.joint_trajectory_topic, 10);
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu joint parameters from rosparam", joint_mappings.size());
  // }

  // Load pose parameters
  // if (this->has_parameter("control_poses")) {
    this->get_parameter("control_poses.pose_list", pose_list);
    for (const auto& pose_name : pose_list) {
      pm.pose_name = pose_name;
      this->get_parameter("control_poses.trigger",                  pm.trigger);
      this->get_parameter("control_poses." + pose_name + ".button", pm.button);

      pose_mappings.push_back(pm);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose parameters from rosparam", pose_mappings.size());
  // }

  // Load cmd_vel parameters
  // if (this->has_parameter("control_velocity")) {
    this->get_parameter("control_velocity.button",             cvm.button);
    this->get_parameter("control_velocity.fast_button",        cvm.fast_button);
    this->get_parameter("control_velocity.axis",               cvm.axis);
    this->get_parameter("control_velocity.fast_axis",          cvm.fast_axis);
    this->get_parameter("control_velocity.linear_x_axis",      cvm.linear_x_axis);
    this->get_parameter("control_velocity.linear_y_axis",      cvm.linear_y_axis);
    this->get_parameter("control_velocity.angular_axis",       cvm.angular_axis);
    this->get_parameter("control_velocity.axis_sign",          cvm.axis_sign);
    this->get_parameter("control_velocity.linear_scale",       cvm.linear_scale);
    this->get_parameter("control_velocity.angular_scale",      cvm.angular_scale);
    this->get_parameter("control_velocity.fast_linear_scale",  cvm.fast_linear_scale);
    this->get_parameter("control_velocity.fast_angular_scale", cvm.fast_angular_scale);
    RCLCPP_INFO(get_logger(), "Loaded control_velocity parameters from rosparam");
  // }
}

void SOBITSTeleop::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
  joint_pos[msg->name[i]] = msg->position[i];
  }
  joint_state_initialized = true;
}

void SOBITSTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  previous_buttons = latest_buttons;
  latest_axes      = msg->axes;
  latest_buttons   = msg->buttons;
  joy_received     = true;
}


void SOBITSTeleop::teleop()
{
  if (!joint_state_initialized) return;
  if (!joy_received) return;

  std::map<std::string, trajectory_msgs::msg::JointTrajectory> trajs;

  for (auto &[name, m] : joint_mappings) {

    if (latest_buttons[m.button] == 0) continue;

    float axis_val = latest_axes[m.axis];
    if (std::abs(axis_val) < 1e-3) continue;

    double delta_pos = axis_val * m.axis_sign * (latest_buttons[m.fast_button] == 1 ? m.fast_speed : m.speed);
    joint_pos[m.joint_name] += delta_pos;
    joint_pos[m.joint_name] = std::clamp(joint_pos[m.joint_name], m.min_pos, m.max_pos);

    auto &traj = trajs[m.joint_trajectory_topic];
    traj.joint_names.push_back(m.joint_name);
    if (traj.points.empty()) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = {joint_pos[m.joint_name]};
      p.time_from_start = rclcpp::Duration::from_seconds(dt);
      traj.points.push_back(p);
    }
    else traj.points[0].positions.push_back(joint_pos[m.joint_name]);
  }

  for (auto &tj : trajs) {
    const auto &joint_trajectory_topic = tj.first;
    auto &traj = tj.second;
    auto it = joint_pub.find(joint_trajectory_topic);
    if (it != joint_pub.end() && traj.joint_names.size() > 0) it->second->publish(traj);
  }

  for (const auto &pose_map : pose_mappings) {
    if (latest_buttons[pose_map.trigger] == 0) continue;
    
    bool button_just_pressed = pose_map.button >= 0 && 
                              pose_map.button < static_cast<int>(latest_buttons.size()) &&
                              latest_buttons[pose_map.button] == 1 &&
                              (previous_buttons.empty() || 
                               pose_map.button >= static_cast<int>(previous_buttons.size()) || 
                               previous_buttons[pose_map.button] == 0);
                               
    if (!button_just_pressed) continue;
    
    if (!move_to_pose_client->wait_for_action_server(std::chrono::seconds(1))) continue;
    
    auto goal_msg = sobits_interfaces::action::MoveToPose::Goal();
    goal_msg.pose_name = pose_map.pose_name;
    goal_msg.time_allowance.sec = 10;
    
    auto send_goal_options = rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this, pose_name = pose_map.pose_name](const auto &result) {
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
          RCLCPP_ERROR(get_logger(), "Unknown result code for pose '%s'", pose_name.c_str());
          break;
      }
    };
    move_to_pose_client->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "Sending pose: %s", pose_map.pose_name.c_str());
  }

  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Twist stop;
  
  bool cmd_vel_enabled = false;
  bool fast_mode = false;
  
  if (cvm.button >= 0 && 
      cvm.button < static_cast<int>(latest_buttons.size())) {
    cmd_vel_enabled = (latest_buttons[cvm.button] == 1);
    if (cvm.fast_button >= 0 && 
        cvm.fast_button < static_cast<int>(latest_buttons.size())) {
      fast_mode = (latest_buttons[cvm.fast_button] == 1);
    }
  }
  else if (cvm.axis >= 0 && 
           cvm.axis < static_cast<int>(latest_axes.size())) {
    cmd_vel_enabled = (latest_axes[cvm.axis] > 0.5);
    if (cvm.fast_axis >= 0 && 
        cvm.fast_axis < static_cast<int>(latest_axes.size())) {
      fast_mode = (latest_axes[cvm.fast_axis] > 0.5);
    }
  }
  
  if (cmd_vel_enabled) {
    const double linear_scale = fast_mode ? cvm.fast_linear_scale : cvm.linear_scale;
    const double angular_scale = fast_mode ? cvm.fast_angular_scale : cvm.angular_scale;

    twist.linear.x = latest_axes[cvm.linear_x_axis] * linear_scale;
    twist.linear.y = latest_axes[cvm.linear_y_axis] * linear_scale * cvm.axis_sign;
    twist.angular.z = latest_axes[cvm.angular_axis] * angular_scale * cvm.axis_sign;

    cmd_vel_pub->publish(twist);
  }
  else cmd_vel_pub->publish(stop);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SOBITSTeleop>());
  rclcpp::shutdown();
  return 0;
}
