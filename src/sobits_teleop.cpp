#include "sobits_teleop/sobits_teleop.hpp"

namespace sobits_teleop
{

SOBITSTeleop::SOBITSTeleop(const rclcpp::NodeOptions & options)
  : Node(
      "sobits_teleop",
      rclcpp::NodeOptions(options)
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)),
  wall_clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
  tf_buffer(std::make_shared<tf2_ros::Buffer>(
      wall_clock_,
      tf2::Duration(std::chrono::seconds(30))))
{
  move_to_pose_client = rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
      this, "move_to_pose");
  move_joint_client = rclcpp_action::create_client<sobits_interfaces::action::MoveJoint>(
      this, "move_joint");

  joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&SOBITSTeleop::joy_callback, this, std::placeholders::_1));

  // Dynamic TFs: robot (sim-time) + Quest (wall-clock) — re-stamp sim-time ones.
  robot_tf_sub = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100).best_effort(),
    std::bind(&SOBITSTeleop::robot_tf_callback, this, std::placeholders::_1));

  // Static TFs: fixed joints (hand_*_end_effector_link etc.) published once on /tf_static.
  // Use transient-local QoS so we receive the latched message even if we subscribe late.
  robot_tf_static_sub = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    rclcpp::QoS(100).transient_local().reliable(),
    std::bind(&SOBITSTeleop::robot_tf_static_callback, this, std::placeholders::_1));

  robot_description_source_node = "robot_state_publisher";
  async_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, robot_description_source_node);

  urdf_timer = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&SOBITSTeleop::load_joint_limits, this));

  load_parameters();

  if (!this->has_parameter("teleop_rate_hz"))
    this->declare_parameter("teleop_rate_hz", teleop_rate_hz);
  this->get_parameter("teleop_rate_hz", teleop_rate_hz);
  if (teleop_rate_hz <= 0.0) {
    RCLCPP_WARN(get_logger(),
      "teleop_rate_hz=%.2f is invalid — clamping to 20.0 Hz", teleop_rate_hz);
    teleop_rate_hz = 20.0;
  }
  // Config speed values are radians per legacy 50 ms tick (the hardcoded timer
  // period they were tuned against); scale per-tick jog deltas to the actual
  // loop period so raising teleop_rate_hz does not speed up jogging.
  jog_tick_scale_ = (1.0 / teleop_rate_hz) / 0.05;

  timer = create_wall_timer(
    std::chrono::duration<double>(1.0 / teleop_rate_hz),
    std::bind(&SOBITSTeleop::teleop, this));
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void SOBITSTeleop::load_joint_limits()
{
  if (!requires_joint_states) return;
  if (urdf_loaded) return;

  if (!async_param_client->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Parameter service not ready for %s", robot_description_source_node.c_str());
    return;
  }

  if (!robot_desc_requested) {
    robot_desc_future = async_param_client->get_parameters({"robot_description"});
    robot_desc_requested = true;
    return;
  }

  if (robot_desc_future.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
    return;
  }

  std::vector<rclcpp::Parameter> params;
  try {
    params = robot_desc_future.get();
  } catch (...) {
    robot_desc_requested = false;
    return;
  }
  robot_desc_requested = false;

  if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) return;

  const std::string urdf_xml = params[0].as_string();
  if (urdf_xml.empty()) return;

  if (!parse_urdf_limits(urdf_xml)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to parse URDF limits");
    return;
  }

  urdf_loaded = true;
  urdf_timer.reset();
  RCLCPP_INFO(get_logger(), "Joint limits loaded (%zu joints)", joint_limits.size());
}

bool SOBITSTeleop::parse_urdf_limits(const std::string & urdf_xml)
{
  urdf::Model model;

  if (!model.initString(urdf_xml)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
    return false;
  }

  joint_limits.clear();

  for (const auto & joint_pair : model.joints_) {

    const auto & joint = joint_pair.second;

    if (!joint) continue;

    if (joint->limits) {
      lim.lower = joint->limits->lower;
      lim.upper = joint->limits->upper;

      joint_limits[joint->name] = lim;
    }
  }

  return true;
}

void SOBITSTeleop::load_parameters()
{
  this->get_parameter("robot_topic_name.joint_states_topic", joint_states_topic);

  this->get_parameter("robot_topic_name.cmd_vel_topic", cvm.topic);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    cvm.topic, 10);

  // Load joint parameters
  if (this->has_parameter("control_joints.groups")) {
    this->get_parameter("control_joints.groups", joint_groups);

    for (const auto& joint_group : joint_groups) {
      if (!this->get_parameter("control_joints." + joint_group + ".names", joint_names)) continue;
      
      for (const auto& joint_name : joint_names) {
        jm.joint_group = joint_group;
        jm.joint_name  = joint_name;
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".button",      jm.button);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".fast_button", jm.fast_button);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".axis",        jm.axis);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".axis_sign",   jm.axis_sign);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".speed",       jm.speed);
        this->get_parameter("control_joints." + joint_group + "." + joint_name + ".fast_speed",  jm.fast_speed);
        this->get_parameter("robot_topic_name.joint_trajectory_topic." + joint_group,            jm.joint_trajectory_topic);

        joint_mappings[joint_name] = jm;
        joint_pub[jm.joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          jm.joint_trajectory_topic, 10);
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu joint parameters from rosparam", joint_mappings.size());
  }

  // Load pose parameters
  if (this->has_parameter("control_poses.trigger")) {
    this->get_parameter("control_poses.pose_list", pose_list);
    for (const auto& pose_name : pose_list) {
      pm.pose_name = pose_name;
      this->get_parameter("control_poses.trigger",                  pm.trigger);
      this->get_parameter("control_poses." + pose_name + ".button", pm.button);

      pose_mappings.push_back(pm);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose parameters from rosparam", pose_mappings.size());
  }

  // Load cmd_vel parameters. Either button-based or axis-based enable is allowed.
  if (this->has_parameter("control_velocity.button") ||
      this->has_parameter("control_velocity.axis")) {
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
  }
  // Load quest parameters
  if (this->has_parameter("quest_control.controller")) {
    this->get_parameter("quest_control.controller", controller_types);
    for (const auto& controller_type : controller_types) {
      if (controller_type == "head") {
        this->get_parameter("quest_control." + controller_type + ".vertical",             qhm.vertical);
        this->get_parameter("quest_control." + controller_type + ".horizontal",           qhm.horizontal);
        this->get_parameter("quest_control." + controller_type + ".head_mode",            qhm.head_mode);
        this->get_parameter("quest_control." + controller_type + ".vertical_sign",        qhm.vertical_sign);
        this->get_parameter("quest_control." + controller_type + ".horizontal_sign",      qhm.horizontal_sign);
        this->get_parameter("quest_control." + controller_type + ".scale",                qhm.scale);
        this->get_parameter("robot_topic_name.joint_trajectory_topic." + controller_type, qhm.head_joint_trajectory_topic);
        joint_pub[qhm.head_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          qhm.head_joint_trajectory_topic, 10);

        if (this->has_parameter("quest_control." + controller_type + ".body_lift")) {
          this->get_parameter("quest_control." + controller_type + ".body_lift",             qhm.body_lift);
          this->get_parameter("robot_topic_name.joint_trajectory_topic.body",                qhm.body_joint_trajectory_topic);
          joint_pub[qhm.body_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            qhm.body_joint_trajectory_topic, 10);
        }
      }
      else {
        qcm = QuestControllerMap{};
        if (this->has_parameter("quest_control." + controller_type + ".arm")) {
          this->get_parameter("quest_control." + controller_type + ".arm",                     qcm.arm);
          this->get_parameter("quest_control." + controller_type + ".base_frame_name",         qcm.base_frame_name);
          this->get_parameter("quest_control." + controller_type + ".end_effector_frame_name", qcm.end_effector_frame_name);
          this->get_parameter("quest_control." + controller_type + ".target_frame_name",       qcm.target_frame_name);
          this->get_parameter("quest_control." + controller_type + ".scale",                   qcm.scale);
          this->get_parameter("quest_control." + controller_type + ".arm_mode",                qcm.arm_mode);
          this->get_parameter("robot_topic_name.joint_trajectory_topic." + qcm.arm,            qcm.arm_joint_trajectory_topic);
          // Optional proximity thresholds — defaults are set in the struct
          if (this->has_parameter("quest_control." + controller_type + ".arm_proximity_threshold")) {
            this->get_parameter("quest_control." + controller_type + ".arm_proximity_threshold",
              qcm.arm_proximity_threshold);
          }
          if (this->has_parameter("quest_control." + controller_type + ".arm_proximity_angle_threshold")) {
            this->get_parameter("quest_control." + controller_type + ".arm_proximity_angle_threshold",
              qcm.arm_proximity_angle_threshold);
          }

          joint_pub[qcm.arm_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          qcm.arm_joint_trajectory_topic, 10);
        }
        if (this->has_parameter("quest_control." + controller_type + ".gripper.hand")) {
          this->get_parameter("quest_control." + controller_type + ".gripper.hand",            qcm.hand);
          this->get_parameter("quest_control." + controller_type + ".gripper.names",           qcm.names);
          this->get_parameter("quest_control." + controller_type + ".gripper.gripper_mode",    qcm.gripper_mode);
          this->get_parameter("quest_control." + controller_type + ".gripper.axis",            qcm.axis);
          this->get_parameter("quest_control." + controller_type + ".gripper.axis_sign",       qcm.axis_sign);
          this->get_parameter("quest_control." + controller_type + ".gripper.speed",           qcm.speed);
          this->get_parameter("robot_topic_name.joint_trajectory_topic." + qcm.hand,           qcm.hand_joint_trajectory_topic);
          if (this->has_parameter("quest_control." + controller_type + ".gripper.type_axis")) {
            this->get_parameter("quest_control." + controller_type + ".gripper.type_axis",     qcm.type_axis);
            this->get_parameter("quest_control." + controller_type + ".gripper.type_joint",    qcm.type_joint);
          }
          if (this->has_parameter("quest_control." + controller_type + ".gripper.hand_pose_button")) {
            this->get_parameter("quest_control." + controller_type + ".gripper.hand_pose_button", qcm.hand_pose_button);
          }
          if (this->has_parameter("quest_control." + controller_type + ".gripper.hand_pose_open")) {
            this->get_parameter("quest_control." + controller_type + ".gripper.hand_pose_open",   qcm.hand_pose_open);
            this->get_parameter("quest_control." + controller_type + ".gripper.hand_pose_close",  qcm.hand_pose_close);
            this->get_parameter("quest_control." + controller_type + ".gripper.hand_pose_action", qcm.hand_pose_action);
          }
          if (this->has_parameter("quest_control." + controller_type + ".gripper.adaptive_trigger_axis")) {
            this->get_parameter("quest_control." + controller_type + ".gripper.adaptive_trigger_axis", qcm.adaptive_trigger_axis);
            this->get_parameter("quest_control." + controller_type + ".gripper.adaptive_stick_axis",   qcm.adaptive_stick_axis);
            this->get_parameter("quest_control." + controller_type + ".gripper.adaptive_close_sign",   qcm.adaptive_close_sign);

            // Load adaptive joint list: adaptive_joints is a list of joint names,
            // each with close_pos, open_pos, and optional fixed flag.
            const std::string aj_prefix = "quest_control." + controller_type + ".gripper.adaptive_joints";
            if (this->has_parameter(aj_prefix + ".names")) {
              std::vector<std::string> aj_names;
              this->get_parameter(aj_prefix + ".names", aj_names);
              for (const auto & jname : aj_names) {
                AdaptiveJointTarget ajt;
                ajt.name = jname;
                this->get_parameter(aj_prefix + "." + jname + ".close_pos", ajt.close_pos);
                this->get_parameter(aj_prefix + "." + jname + ".open_pos",  ajt.open_pos);
                if (this->has_parameter(aj_prefix + "." + jname + ".fixed")) {
                  this->get_parameter(aj_prefix + "." + jname + ".fixed", ajt.fixed);
                }
                qcm.adaptive_joints.push_back(ajt);
              }
            }
          }
          joint_pub[qcm.hand_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            qcm.hand_joint_trajectory_topic, 10);
        }
        quest_controller_mappings[controller_type] = qcm;
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu quest controller parameters from rosparam", quest_controller_mappings.size());
    has_quest_controls = !controller_types.empty();

    // Create one enable-publisher per arm (planning group)
    for (const auto & [ctrl_name, qcm_ref] : quest_controller_mappings) {
      if (!qcm_ref.arm.empty() && arm_track_pubs_.find(qcm_ref.arm) == arm_track_pubs_.end()) {
        // Reliable + transient_local (depth 1) so a late-starting subscriber
        // (e.g. the Servo target bridge, which may come up after this node)
        // still receives the current enable state instead of missing it.
        // moveit_arm_controller's volatile subscriber remains compatible: QoS
        // compatibility only requires publisher-durability >= subscriber-durability.
        arm_track_pubs_[qcm_ref.arm] = this->create_publisher<std_msgs::msg::Bool>(
          qcm_ref.arm + "/moveit_track_enabled",
          rclcpp::QoS(1).reliable().transient_local());
        RCLCPP_INFO(get_logger(),
          "Created arm track publisher for '%s'", qcm_ref.arm.c_str());
      }
      // Create hand pose action client if configured
      if (!qcm_ref.hand_pose_action.empty() &&
          hand_pose_clients_.find(ctrl_name) == hand_pose_clients_.end()) {
        hand_pose_clients_[ctrl_name] =
          rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
            this, qcm_ref.hand_pose_action);
        hand_open_state_[ctrl_name]  = true;
        hand_toggle_time_[ctrl_name] = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(get_logger(),
          "Created hand pose client for '%s' → '%s'",
          ctrl_name.c_str(), qcm_ref.hand_pose_action.c_str());
      }
    }
  }

  requires_joint_states = !joint_mappings.empty() || has_quest_controls;

  if (requires_joint_states && joint_states_topic.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "joint_states_topic is required for joint or quest teleop, disabling those controls.");
    joint_mappings.clear();
    quest_controller_mappings.clear();
    controller_types.clear();
    has_quest_controls = false;
    requires_joint_states = false;
  }

  if (!joint_states_topic.empty()) {
    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, 10,
      std::bind(&SOBITSTeleop::joint_state_callback, this, std::placeholders::_1));
  }
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


void SOBITSTeleop::robot_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // tf_buffer uses wall-clock time. Quest TFs arrive wall-clock stamped (~1.776e9 s)
  // and go in as-is. Robot TFs arrive sim-time stamped (~300 s); re-stamp them with
  // the current wall-clock time so tf2 does not reject them as "old data".
  const rclcpp::Time now_wall = wall_clock_->now();
  constexpr int64_t kSimTimeThresholdSec = 1'000'000'000LL;  // < 1e9 s → sim-time

  for (const auto & t : msg->transforms) {
    geometry_msgs::msg::TransformStamped ts = t;
    if (ts.header.stamp.sec < kSimTimeThresholdSec) {
      ts.header.stamp = now_wall;
    }
    try {
      tf_buffer->setTransform(ts, "tf", false);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *wall_clock_, 2000,
        "tf_buffer setTransform failed for %s: %s", ts.child_frame_id.c_str(), ex.what());
    }
  }
}

void SOBITSTeleop::robot_tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // Static TFs (fixed joints) are published once on /tf_static with sim-time stamps.
  // Insert them as static transforms (is_static=true) so they persist in the buffer.
  const rclcpp::Time now_wall = wall_clock_->now();
  constexpr int64_t kSimTimeThresholdSec = 1'000'000'000LL;

  for (const auto & t : msg->transforms) {
    geometry_msgs::msg::TransformStamped ts = t;
    if (ts.header.stamp.sec < kSimTimeThresholdSec) {
      ts.header.stamp = now_wall;
    }
    try {
      tf_buffer->setTransform(ts, "tf_static", true);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *wall_clock_, 2000,
        "tf_buffer setTransform (static) failed for %s: %s", ts.child_frame_id.c_str(), ex.what());
    }
  }
}


void SOBITSTeleop::teleop()
{
  if (!joy_received) return;
  if (requires_joint_states && !joint_state_initialized) return;

  std::map<std::string, trajectory_msgs::msg::JointTrajectory> trajs;

  for (auto &[name, m] : joint_mappings) {

    if (latest_buttons[m.button] == 0) continue;

    float axis_val = latest_axes[m.axis];
    if (std::abs(axis_val) < 1e-3) continue;

    // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
    double delta_pos = axis_val * m.axis_sign * (latest_buttons[m.fast_button] == 1 ? m.fast_speed : m.speed) * jog_tick_scale_;
    joint_pos[m.joint_name] += delta_pos;
    auto [actual_min, actual_max] = std::minmax({joint_limits[m.joint_name].lower, joint_limits[m.joint_name].upper});
    joint_pos[m.joint_name] = std::clamp(joint_pos[m.joint_name], actual_min, actual_max);

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
  if (cvm.axis >= 0 && 
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

    if (cvm.linear_x_axis >= 0 &&
        cvm.linear_x_axis < static_cast<int>(latest_axes.size())) {
      twist.linear.x = latest_axes[cvm.linear_x_axis] * linear_scale;
    }
    if (cvm.linear_y_axis >= 0 &&
        cvm.linear_y_axis < static_cast<int>(latest_axes.size())) {
      twist.linear.y = latest_axes[cvm.linear_y_axis] * linear_scale * cvm.axis_sign;
    }
    if (cvm.angular_axis >= 0 &&
        cvm.angular_axis < static_cast<int>(latest_axes.size())) {
      twist.angular.z = latest_axes[cvm.angular_axis] * angular_scale * cvm.axis_sign;
    }

    cmd_vel_pub->publish(twist);
  }
  else cmd_vel_pub->publish(stop);

  // Quest controllers
  // Unity publishes Quest frames directly under base_footprint, so we look them
  // up from base_footprint. No odom intermediate is needed.
  bool base_odom_ok = true;  // always ready; kept as guard variable for structure

  // out      = T(base_footprint <- quest_frame)  — used for arm target computation
  // out_base = T(base_footprint <- quest_frame)  — used for RViz re-broadcast under base_footprint
  auto lookup_quest_frame = [&](const std::string & quest_frame,
                                tf2::Transform & out,
                                tf2::Transform * out_base = nullptr) -> bool {
    try {
      auto ts = tf_buffer->lookupTransform("base_footprint", quest_frame, tf2::TimePointZero, tf2::Duration(0));
      tf2::Transform T_base_quest;
      tf2::fromMsg(ts.transform, T_base_quest);
      out = T_base_quest;
      if (out_base) *out_base = T_base_quest;
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "%s TF lookup failed: %s", quest_frame.c_str(), ex.what());
      return false;
    }
  };

  if (this->has_parameter("quest_control.controller")) {
    // Head / HMD — also used as body reference for arm target scaling
    bool head_tf_ok = false;
    if (base_odom_ok) {
      tf2::Transform T_base_hmd;
      head_tf_ok = lookup_quest_frame("hmd_odom", current_tf, &T_base_hmd);
      if (head_tf_ok) {
        current_tf_hmd      = current_tf;
        current_tf_hmd_odom = T_base_hmd;
        // Re-broadcast under base_footprint (RViz visualization).
        geometry_msgs::msg::TransformStamped hmd_msg;
        hmd_msg.header.stamp    = this->now();
        hmd_msg.header.frame_id = "base_footprint";
        hmd_msg.child_frame_id  = "hmd_link";
        hmd_msg.transform       = tf2::toMsg(T_base_hmd);
        tf_broadcaster->sendTransform(hmd_msg);
      }
    }

    // --- (hold) ---
    if (head_tf_ok) {
    if (qhm.head_mode >= 0 &&
        qhm.head_mode < static_cast<int>(latest_axes.size())){
          head_control_enabled = (latest_axes[qhm.head_mode] > 0.5);
        }

    if (head_control_enabled && !head_tracking) {
      last_pan = joint_pos[qhm.horizontal];
      last_tilt = joint_pos[qhm.vertical];
      last_tf = current_tf;
      if (!qhm.body_lift.empty()) last_body_lift = joint_pos[qhm.body_lift];
      head_tracking = true;

      // RCLCPP_INFO(this->get_logger(), "cur_joint_pos %.2f, %.2f", joint_pos[qhm.horizontal], joint_pos[qhm.vertical]);
      RCLCPP_INFO(this->get_logger(), "Head tracking started");
    }

    if (head_tracking) {
      T_delta = last_tf.inverse() * current_tf;
      tf2::Matrix3x3(T_delta.getRotation()).getRPY(roll, pitch, yaw);

      pan_target = last_pan + qhm.scale * yaw * qhm.horizontal_sign;
      tilt_target = last_tilt + qhm.scale * pitch * -qhm.vertical_sign;

      pan_target = std::clamp(pan_target, joint_limits[qhm.horizontal].lower, joint_limits[qhm.horizontal].upper);
      tilt_target = std::clamp(tilt_target, joint_limits[qhm.vertical].lower, joint_limits[qhm.vertical].upper);
      // RCLCPP_INFO(this->get_logger(), "pub_joint_pos %.2f, %.2f", pan_target, tilt_target);
      
      if (!qhm.body_lift.empty()) {
        dz = T_delta.getOrigin().z();
        body_lift_target = last_body_lift + qhm.scale * dz;
        body_lift_target = std::clamp(body_lift_target, joint_limits[qhm.body_lift].lower, joint_limits[qhm.body_lift].upper);
      }
      
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {qhm.horizontal, qhm.vertical};

      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = {pan_target, tilt_target};
      p.time_from_start = rclcpp::Duration::from_seconds(dt);
      traj.points.push_back(p);

      auto it = joint_pub.find(qhm.head_joint_trajectory_topic);
      if (it != joint_pub.end() && it->second) {
        it->second->publish(traj);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Publisher for %s not found", qhm.head_joint_trajectory_topic.c_str());
      }

      if (!qhm.body_lift.empty()) {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {qhm.body_lift};

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = {body_lift_target};
        p.time_from_start = rclcpp::Duration::from_seconds(dt);
        traj.points.push_back(p);

        auto it = joint_pub.find(qhm.body_joint_trajectory_topic);
        if (it != joint_pub.end() && it->second) {
          it->second->publish(traj);
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                              "Publisher for %s not found", qhm.body_joint_trajectory_topic.c_str());
        }
      }
    }
    // --- (release) ---
    if (!head_control_enabled && head_tracking) {
      head_tracking = false;
      RCLCPP_INFO(this->get_logger(), "Head tracking stopped");
    }
    } // if (head_tf_ok)


    // Arm
    // Helper: returns false if any component of the transform is NaN/Inf
    // (Quest controllers broadcast NaN when not yet tracked)
    auto transform_valid = [](const geometry_msgs::msg::Transform & t) {
      const auto & q = t.rotation;
      const auto & v = t.translation;
      return std::isfinite(q.x) && std::isfinite(q.y) &&
             std::isfinite(q.z) && std::isfinite(q.w) &&
             std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
             (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) > 0.01;
    };

    bool right_tf_ok = false;
    if (base_odom_ok) {
      tf2::Transform T_right, T_base_right;
      if (lookup_quest_frame("right_controller_odom", T_right, &T_base_right)) {
        geometry_msgs::msg::Transform t_msg = tf2::toMsg(T_right);
        if (!transform_valid(t_msg)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "right_controller_odom has invalid (NaN/zero) transform — waiting for controller tracking");
        } else {
          current_tf_r      = T_right;
          current_tf_r_odom = T_base_right;
          right_tf_ok = true;
          // Re-broadcast under base_footprint (RViz visualization).
          geometry_msgs::msg::TransformStamped rc_msg;
          rc_msg.header.stamp    = this->now();
          rc_msg.header.frame_id = "base_footprint";
          rc_msg.child_frame_id  = "right_controller_link";
          rc_msg.transform       = tf2::toMsg(T_base_right);
          tf_broadcaster->sendTransform(rc_msg);
        }
      }
    }

    bool left_tf_ok = false;
    if (base_odom_ok) {
      tf2::Transform T_left, T_base_left;
      if (lookup_quest_frame("left_controller_odom", T_left, &T_base_left)) {
        geometry_msgs::msg::Transform t_msg = tf2::toMsg(T_left);
        if (!transform_valid(t_msg)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "left_controller_odom has invalid (NaN/zero) transform — waiting for controller tracking");
        } else {
          current_tf_l      = T_left;
          current_tf_l_odom = T_base_left;
          left_tf_ok = true;
          // Re-broadcast under base_footprint (RViz visualization).
          geometry_msgs::msg::TransformStamped lc_msg;
          lc_msg.header.stamp    = this->now();
          lc_msg.header.frame_id = "base_footprint";
          lc_msg.child_frame_id  = "left_controller_link";
          lc_msg.transform       = tf2::toMsg(T_base_left);
          tf_broadcaster->sendTransform(lc_msg);
        }
      }
    }

    // Determine arm_control_enabled from the first arm with a valid arm_mode
    // (both left and right share the same button in the default quest.yaml)
    for (auto &[name, m] : quest_controller_mappings) {
      if (m.arm_mode >= 0 && m.arm_mode < static_cast<int>(latest_axes.size())) {
        arm_control_enabled = (latest_axes[m.arm_mode] > 0.5);
        break;
      }
    }

    // Stop tracking immediately when the grip button is released.
    if (!arm_control_enabled && arm_tracking) {
      arm_tracking = false;
      right_arm_latched = false;
      left_arm_latched  = false;
      RCLCPP_INFO(this->get_logger(), "Arm tracking stopped");
      std_msgs::msg::Bool track_msg;
      track_msg.data = false;
      for (auto & [arm_name, pub] : arm_track_pubs_) {
        pub->publish(track_msg);
      }
    }
    // Per-arm latching is handled inside the per-arm loop below, after the
    // fresh end-effector TF has been read and the proximity check can be done.

    for (auto &[name, m] : quest_controller_mappings) {
      bool gripper_control_enabled = false;
      if (!m.hand.empty()) {
        if (m.gripper_mode >= 0 &&
          m.gripper_mode < static_cast<int>(latest_axes.size())){
            gripper_control_enabled = (latest_axes[m.gripper_mode] > 0.5);
        }
      }

      if (name == "right" && right_tf_ok && head_tf_ok) {
        bool ee_r_ok = false;
        try {
          tf_msg = tf_buffer->lookupTransform(
            "base_footprint",
            m.end_effector_frame_name,
              tf2::TimePointZero,
              tf2::Duration(0)
          );
          tf2::fromMsg(tf_msg.transform, current_tf_ee_r);
          ee_r_ok = true;
        }
        catch (tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Right EE TF lookup failed: %s", ex.what());
        }

        // Per-arm latch: requires grip button. Proximity check is skipped when both
        // thresholds are 0 (immediate latch). T_delta starts as identity so the arm
        // stays at its current pose until the controller actually moves.
        if (arm_control_enabled && !right_arm_latched && ee_r_ok) {
          bool prox_ok = true;
          if (m.arm_proximity_threshold > 0.0 || m.arm_proximity_angle_threshold > 0.0) {
            tf2::Vector3 pos_diff = current_tf_r.getOrigin() - current_tf_ee_r.getOrigin();
            double pos_err = pos_diff.length();
            tf2::Quaternion q_diff =
              current_tf_ee_r.getRotation().inverse() * current_tf_r.getRotation();
            q_diff.normalize();
            double angle_err = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

            if (m.arm_proximity_threshold > 0.0 && pos_err > m.arm_proximity_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Right arm: controller %.3f m from EE (threshold %.3f m) — move controller to EE before gripping",
                pos_err, m.arm_proximity_threshold);
              prox_ok = false;
            }
            if (prox_ok && m.arm_proximity_angle_threshold > 0.0 &&
                angle_err > m.arm_proximity_angle_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Right arm: controller %.1f deg from EE orientation (threshold %.1f deg) — align controller before gripping",
                angle_err * 180.0 / M_PI,
                m.arm_proximity_angle_threshold * 180.0 / M_PI);
              prox_ok = false;
            }
          }
            if (prox_ok) {
            right_arm_latched = true;
            const char * prox_note = (m.arm_proximity_threshold <= 0.0 &&
                                      m.arm_proximity_angle_threshold <= 0.0)
                                     ? " (calibration skipped)" : "";
            if (!arm_tracking) {
              arm_tracking = true;
              RCLCPP_INFO(this->get_logger(), "Arm tracking started (right latched%s)", prox_note);
              std_msgs::msg::Bool track_msg;
              track_msg.data = true;
              for (auto & [arm_name, pub] : arm_track_pubs_) {
                pub->publish(track_msg);
              }
            } else {
              RCLCPP_INFO(this->get_logger(), "Right arm latched%s", prox_note);
            }
          }
        }

        // Compute target in base_footprint space (HMD + scaled controller delta from HMD).
        {
          tf2::Vector3 hmd_pos_odom = current_tf_hmd_odom.getOrigin();
          tf2::Vector3 delta_odom   = current_tf_r_odom.getOrigin() - hmd_pos_odom;
          T_target_r.setOrigin(hmd_pos_odom + delta_odom * m.scale);
          T_target_r.setRotation(current_tf_r_odom.getRotation());
        }

        // Publish target under base_footprint (visualization — stays fixed in robot space).
        target_msg_r.header.stamp    = this->now();
        target_msg_r.header.frame_id = "base_footprint";
        target_msg_r.child_frame_id  = m.target_frame_name;
        target_msg_r.transform       = tf2::toMsg(T_target_r);
        tf_broadcaster->sendTransform(target_msg_r);
      }

      if (name == "left" && left_tf_ok && head_tf_ok) {
        bool ee_l_ok = false;
        try {
          tf_msg = tf_buffer->lookupTransform(
            "base_footprint",
            m.end_effector_frame_name,
              tf2::TimePointZero,
              tf2::Duration(0)
          );
          tf2::fromMsg(tf_msg.transform, current_tf_ee_l);
          ee_l_ok = true;
        }
        catch (tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Left EE TF lookup failed: %s", ex.what());
        }

        // Per-arm latch: requires grip button. Proximity check is skipped when both
        // thresholds are 0 (immediate latch). T_delta starts as identity so the arm
        // stays at its current pose until the controller actually moves.
        if (arm_control_enabled && !left_arm_latched && ee_l_ok) {
          bool prox_ok = true;
          if (m.arm_proximity_threshold > 0.0 || m.arm_proximity_angle_threshold > 0.0) {
            tf2::Vector3 pos_diff = current_tf_l.getOrigin() - current_tf_ee_l.getOrigin();
            double pos_err = pos_diff.length();
            tf2::Quaternion q_diff =
              current_tf_ee_l.getRotation().inverse() * current_tf_l.getRotation();
            q_diff.normalize();
            double angle_err = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

            if (m.arm_proximity_threshold > 0.0 && pos_err > m.arm_proximity_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Left arm: controller %.3f m from EE (threshold %.3f m) — move controller to EE before gripping",
                pos_err, m.arm_proximity_threshold);
              prox_ok = false;
            }
            if (prox_ok && m.arm_proximity_angle_threshold > 0.0 &&
                angle_err > m.arm_proximity_angle_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Left arm: controller %.1f deg from EE orientation (threshold %.1f deg) — align controller before gripping",
                angle_err * 180.0 / M_PI,
                m.arm_proximity_angle_threshold * 180.0 / M_PI);
              prox_ok = false;
            }
          }
          if (prox_ok) {
            left_arm_latched = true;
            const char * prox_note = (m.arm_proximity_threshold <= 0.0 &&
                                      m.arm_proximity_angle_threshold <= 0.0)
                                     ? " (calibration skipped)" : "";
            if (!arm_tracking) {
              arm_tracking = true;
              RCLCPP_INFO(this->get_logger(), "Arm tracking started (left latched%s)", prox_note);
              std_msgs::msg::Bool track_msg;
              track_msg.data = true;
              for (auto & [arm_name, pub] : arm_track_pubs_) {
                pub->publish(track_msg);
              }
            } else {
              RCLCPP_INFO(this->get_logger(), "Left arm latched%s", prox_note);
            }
          }
        }

        // Compute target in base_footprint space.
        {
          tf2::Vector3 hmd_pos_odom = current_tf_hmd_odom.getOrigin();
          tf2::Vector3 delta_odom   = current_tf_l_odom.getOrigin() - hmd_pos_odom;
          T_target_l.setOrigin(hmd_pos_odom + delta_odom * m.scale);
          T_target_l.setRotation(current_tf_l_odom.getRotation());
        }

        // Publish target under base_footprint (visualization).
        target_msg_l.header.stamp    = this->now();
        target_msg_l.header.frame_id = "base_footprint";
        target_msg_l.child_frame_id  = m.target_frame_name;
        target_msg_l.transform       = tf2::toMsg(T_target_l);
        tf_broadcaster->sendTransform(target_msg_l);
      }

      // Gripper
      if (!m.hand.empty()) {
        // ── 1. Hand pose toggle (open / close) on button press ───────────────
        // Configured via gripper.hand_pose_button / hand_pose_open /
        // hand_pose_close / hand_pose_action in quest.yaml.
        auto hp_client_it = hand_pose_clients_.find(name);
        if (m.hand_pose_button >= 0 && hp_client_it != hand_pose_clients_.end()) {
          rclcpp::Time & toggle_time = hand_toggle_time_.at(name);
          const bool debounce_ok = (this->now() - toggle_time).seconds() > 0.4;

          if (m.hand_pose_button < static_cast<int>(latest_buttons.size()) &&
              latest_buttons[m.hand_pose_button] == 1 &&
              (previous_buttons.empty() ||
               m.hand_pose_button >= static_cast<int>(previous_buttons.size()) ||
               previous_buttons[m.hand_pose_button] == 0) &&
              debounce_ok)
          {
            toggle_time = this->now();
            bool & is_open = hand_open_state_.at(name);
            is_open = !is_open;
            const std::string pose_name = is_open ? m.hand_pose_open : m.hand_pose_close;

            auto & client = hp_client_it->second;
            if (client->wait_for_action_server(std::chrono::milliseconds(200))) {
              auto goal = sobits_interfaces::action::MoveToPose::Goal();
              goal.pose_name = pose_name;
              // joint_action_server uses time_allowance as the trajectory's
              // time_from_start, so this IS the open/close motion duration —
              // 5 s made the gripper crawl. 1 s is ample for the hand joints.
              goal.time_allowance.sec = 1;
              auto opts = rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SendGoalOptions();
              opts.result_callback = [this, pose_name](const auto & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                  RCLCPP_INFO(get_logger(), "Hand pose '%s' succeeded", pose_name.c_str());
                else
                  RCLCPP_WARN(get_logger(), "Hand pose '%s' failed", pose_name.c_str());
              };
              client->async_send_goal(goal, opts);
              RCLCPP_INFO(get_logger(), "%s hand → %s", name.c_str(), pose_name.c_str());
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "'%s' hand pose action server not available", name.c_str());
            }
          }
        }

        // ── 2. Adaptive gripper control (trigger held + thumbstick) ──────────
        // Configured via gripper.adaptive_trigger_axis / adaptive_stick_axis /
        // adaptive_close_sign / adaptive_joints in quest.yaml.
        // adaptive_close_sign: +1 → positive stick = close, -1 → negative = close.
        if (m.adaptive_trigger_axis >= 0 && !m.adaptive_joints.empty() &&
            m.adaptive_trigger_axis < static_cast<int>(latest_axes.size()) &&
            m.adaptive_stick_axis   < static_cast<int>(latest_axes.size()) &&
            latest_axes[m.adaptive_trigger_axis] > 0.1)
        {
          float raw_stick  = latest_axes[m.adaptive_stick_axis];
          float close_frac = std::clamp( raw_stick * m.adaptive_close_sign, 0.0f, 1.0f);
          float open_frac  = std::clamp(-raw_stick * m.adaptive_close_sign, 0.0f, 1.0f);
          float deflection = std::max(close_frac, open_frac);

          if (deflection >= 0.1f) {
            const bool closing = (close_frac >= open_frac);
            // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
            float step = m.speed * deflection * static_cast<float>(jog_tick_scale_);

            auto step_toward = [&](const std::string & jname, float target) -> float {
              if (joint_pos.find(jname) == joint_pos.end()) return target;
              float cur = static_cast<float>(joint_pos.at(jname));
              float dir = (target > cur) ? 1.0f : -1.0f;
              float next = cur + dir * step;
              if ((dir > 0 && next > target) || (dir < 0 && next < target)) next = target;
              if (joint_limits.count(jname)) {
                next = std::clamp(next,
                  static_cast<float>(std::min(joint_limits.at(jname).lower, joint_limits.at(jname).upper)),
                  static_cast<float>(std::max(joint_limits.at(jname).lower, joint_limits.at(jname).upper)));
              }
              return next;
            };

            auto goal = sobits_interfaces::action::MoveJoint::Goal();
            for (const auto & ajt : m.adaptive_joints) {
              goal.target_joint_names.push_back(ajt.name);
              if (ajt.fixed) {
                goal.target_joint_rad.push_back(ajt.close_pos);
              } else {
                float tgt = closing ? ajt.close_pos : ajt.open_pos;
                goal.target_joint_rad.push_back(step_toward(ajt.name, tgt));
              }
            }
            goal.time_allowance.nanosec = static_cast<uint32_t>(100'000'000u);  // 100 ms

            if (move_joint_client->action_server_is_ready()) {
              auto opts = rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SendGoalOptions();
              move_joint_client->async_send_goal(goal, opts);
            }
          }
        }

        // ── 3. Continuous gripper via joint trajectory publisher ─────────────
        if (gripper_control_enabled) {
          trajectory_msgs::msg::JointTrajectory traj;
          trajectory_msgs::msg::JointTrajectoryPoint p;

          for (const auto & joint_name : m.names) {
            if (std::abs(latest_axes[m.axis]) > 0.2) {
              // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
              target_rad = joint_pos[joint_name] + m.speed * latest_axes[m.axis] * m.axis_sign * jog_tick_scale_;
              target_rad = std::clamp(target_rad,
                std::min(joint_limits[joint_name].lower, joint_limits[joint_name].upper),
                std::max(joint_limits[joint_name].lower, joint_limits[joint_name].upper));
              traj.joint_names.push_back(joint_name);
              p.positions.push_back(target_rad);
            }
          }

          if (m.type_axis >= 0 && m.type_axis < static_cast<int>(latest_axes.size()) &&
              std::abs(latest_axes[m.type_axis]) > 0.8) {
            target_rad = joint_pos[m.type_joint] + m.speed * std::copysign(1.0, latest_axes[m.type_axis]) * -m.axis_sign * jog_tick_scale_;
            target_rad = std::clamp(target_rad,
              std::min(joint_limits[m.type_joint].lower, joint_limits[m.type_joint].upper),
              std::max(joint_limits[m.type_joint].lower, joint_limits[m.type_joint].upper));
            traj.joint_names.push_back(m.type_joint);
            p.positions.push_back(target_rad);
          }

          if (!traj.joint_names.empty()) {
            traj.points.push_back(p);
            auto it = joint_pub.find(m.hand_joint_trajectory_topic);
            if (it != joint_pub.end() && it->second) {
              it->second->publish(traj);
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Publisher for %s not found", m.hand_joint_trajectory_topic.c_str());
            }
          }
        }
      }// Gripper
    }// Arm
  }// Quest controllers

}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::SOBITSTeleop)
