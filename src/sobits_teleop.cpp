#include "sobits_teleop/sobits_teleop.hpp"

SOBITSTeleop::SOBITSTeleop()
  : Node(
      "sobits_teleop",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)),
  tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer))
{
  move_to_pose_client = rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
      this, "move_to_pose");

  joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&SOBITSTeleop::joy_callback, this, std::placeholders::_1));

  async_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "robot_state_publisher");

  urdf_timer = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&SOBITSTeleop::load_joint_limits, this));

  load_parameters();

  timer = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SOBITSTeleop::teleop, this));
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void SOBITSTeleop::load_joint_limits()
{
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
  joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, 10,
    std::bind(&SOBITSTeleop::joint_state_callback, this, std::placeholders::_1));

  this->get_parameter("robot_topic_name.cmd_vel_topic", cvm.topic);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    cvm.topic, 10);

  // Load joint parameters
  if (this->has_parameter("control_joints")) {
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
  if (this->has_parameter("control_poses")) {
    this->get_parameter("control_poses.pose_list", pose_list);
    for (const auto& pose_name : pose_list) {
      pm.pose_name = pose_name;
      this->get_parameter("control_poses.trigger",                  pm.trigger);
      this->get_parameter("control_poses." + pose_name + ".button", pm.button);

      pose_mappings.push_back(pm);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose parameters from rosparam", pose_mappings.size());
  }

  // Load cmd_vel parameters
  if (this->has_parameter("control_velocity.axis")) {
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
        if (this->has_parameter("quest_control." + controller_type + ".arm")) {
          this->get_parameter("quest_control." + controller_type + ".arm",                     qcm.arm);
          this->get_parameter("quest_control." + controller_type + ".base_frame_name",         qcm.base_frame_name);
          this->get_parameter("quest_control." + controller_type + ".end_effector_frame_name", qcm.end_effector_frame_name);
          this->get_parameter("quest_control." + controller_type + ".target_frame_name",       qcm.target_frame_name);
          this->get_parameter("quest_control." + controller_type + ".scale",                   qcm.scale);
          this->get_parameter("quest_control." + controller_type + ".arm_mode",                qcm.arm_mode);
          this->get_parameter("robot_topic_name.joint_trajectory_topic." + qcm.arm,            qcm.arm_joint_trajectory_topic);

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
          joint_pub[qcm.hand_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            qcm.hand_joint_trajectory_topic, 10);
        }
        quest_controller_mappings[controller_type] = qcm;
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu quest controller parameters from rosparam", quest_controller_mappings.size());
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

    twist.linear.x = latest_axes[cvm.linear_x_axis] * linear_scale;
    twist.linear.y = latest_axes[cvm.linear_y_axis] * linear_scale * cvm.axis_sign;
    twist.angular.z = latest_axes[cvm.angular_axis] * angular_scale * cvm.axis_sign;

    cmd_vel_pub->publish(twist);
  }
  else cmd_vel_pub->publish(stop);

    
  // Head
  try {
      tf_msg = tf_buffer->lookupTransform(
        std::string(this->get_namespace()).substr(1) + "/base_footprint",
        "hmd_odom",
        tf2::TimePointZero
      );
      tf2::fromMsg(tf_msg.transform, current_tf);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

  // --- (hold) ---
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


  // Arm
  try {
    tf_msg = tf_buffer->lookupTransform(
      std::string(this->get_namespace()).substr(1) + "/base_footprint",
      "right_controller_odom",
      tf2::TimePointZero
    );
    tf2::fromMsg(tf_msg.transform, current_tf_r);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }
  
  try {
    tf_msg = tf_buffer->lookupTransform(
      std::string(this->get_namespace()).substr(1) + "/base_footprint",
      "left_controller_odom",
      tf2::TimePointZero
    );
    tf2::fromMsg(tf_msg.transform, current_tf_l);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  q_align.setRPY(0.0, -M_PI_4, 0.0);   // pitch -45deg
  T_align.setOrigin(tf2::Vector3(0,0,0));
  T_align.setRotation(q_align);

  for (auto &[name, m] : quest_controller_mappings) {
    if (m.arm_mode >= 0 &&
      m.arm_mode < static_cast<int>(latest_axes.size())){
        arm_control_enabled = (latest_axes[m.arm_mode] > 0.5);
    }
    if (!m.hand.empty()) {
      if (m.gripper_mode >= 0 &&
        m.gripper_mode < static_cast<int>(latest_axes.size())){
          gripper_control_enabled = (latest_axes[m.gripper_mode] > 0.5);
      }
    }

    if (arm_control_enabled && !arm_tracking) {
      last_tf_r = current_tf_r;
      last_tf_ee_r = current_tf_ee_r;
      last_tf_l = current_tf_l;
      last_tf_ee_l = current_tf_ee_l;
      arm_tracking = true;
      RCLCPP_INFO(this->get_logger(), "arm tracking started");
    }

    if (!arm_control_enabled && arm_tracking) {
      arm_tracking = false;
      RCLCPP_INFO(this->get_logger(), "arm tracking stopped");
    }

    if (name == "right") {
      try {
        tf_msg = tf_buffer->lookupTransform(
          std::string(this->get_namespace()).substr(1) + "/base_footprint",
          std::string(this->get_namespace()).substr(1) + "/" + m.end_effector_frame_name,
          tf2::TimePointZero
        );
        tf2::fromMsg(tf_msg.transform, current_tf_ee_r);
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
      }

      // --- (right_hold) ---
      if (arm_tracking) {
        T_delta_r = last_tf_r.inverse() * current_tf_r;
        T_delta_r_align = T_align * T_delta_r * T_align.inverse();
        T_target_r = last_tf_ee_r * T_delta_r_align;

        target_msg_r.header.stamp = this->now();
        target_msg_r.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";
        target_msg_r.child_frame_id = std::string(this->get_namespace()).substr(1) + "/" + m.target_frame_name;
        target_msg_r.transform = tf2::toMsg(T_target_r);

        tf_broadcaster->sendTransform(target_msg_r);
      }
    }

    if (name == "left") {
      try {
        tf_msg = tf_buffer->lookupTransform(
          std::string(this->get_namespace()).substr(1) + "/base_footprint",
          std::string(this->get_namespace()).substr(1) + "/" + m.end_effector_frame_name,
          tf2::TimePointZero
        );
        tf2::fromMsg(tf_msg.transform, current_tf_ee_l);
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
      }

      // --- (left_hold) ---
      if (arm_tracking) {
        T_delta_l = last_tf_l.inverse() * current_tf_l;
        T_delta_l_align = T_align * T_delta_l * T_align.inverse();
        T_target_l = last_tf_ee_l * T_delta_l_align;

        target_msg_l.header.stamp = this->now();
        target_msg_l.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";
        target_msg_l.child_frame_id = std::string(this->get_namespace()).substr(1) + "/" + m.target_frame_name;
        target_msg_l.transform = tf2::toMsg(T_target_l);

        tf_broadcaster->sendTransform(target_msg_l);
      }
    }

    // Gripper
    if (!m.hand.empty() && gripper_control_enabled) {
      trajectory_msgs::msg::JointTrajectory traj;
      trajectory_msgs::msg::JointTrajectoryPoint p;

      for (const auto &joint_name : m.names) {
        if (std::abs(latest_axes[m.axis]) > 0.2) {
          if (name == "left") {
            target_rad = joint_pos[joint_name] + m.speed * latest_axes[m.axis] * m.axis_sign;
            if (joint_name == "hand_left_finger_l_pip_joint" || joint_name == "hand_left_finger_l_dip_joint") {
              target_rad = joint_pos[joint_name] + m.speed * latest_axes[m.axis] * -m.axis_sign;
            }
          }
          if (name == "right") {
            target_rad = joint_pos[joint_name] + m.speed * latest_axes[m.axis] * -m.axis_sign;
            if (joint_name == "hand_right_finger_r_pip_joint" || joint_name == "hand_right_finger_r_dip_joint") {
              target_rad = joint_pos[joint_name] + m.speed * latest_axes[m.axis] * m.axis_sign;
            }
          }
          target_rad = std::clamp(target_rad, 
            std::min(joint_limits[joint_name].lower, joint_limits[joint_name].upper), 
            std::max(joint_limits[joint_name].lower, joint_limits[joint_name].upper));

          if (joint_name == "hand_right_finger_c_mcp_joint" || joint_name == "hand_right_finger_c_ip_joint") {
            RCLCPP_INFO(this->get_logger(), "%s target_rad : %.2f", joint_name.c_str(), target_rad);
            RCLCPP_INFO(this->get_logger(), "%s joint_pos : %.2f", joint_name.c_str(), joint_pos[joint_name]);
          }
          traj.joint_names.push_back(joint_name);
          p.positions.push_back(target_rad);
        }
      }

      if (m.type_axis >= 0 && m.type_axis < static_cast<int>(latest_axes.size()) && 
      std::abs(latest_axes[m.type_axis]) > 0.8) {
        target_rad = 0.0;

        target_rad = joint_pos[m.type_joint] + m.speed * std::copysign(1.0, latest_axes[m.type_axis]) * -m.axis_sign;

        target_rad = std::clamp(target_rad, 
          std::min(joint_limits[m.type_joint].lower, joint_limits[m.type_joint].upper), 
          std::max(joint_limits[m.type_joint].lower, joint_limits[m.type_joint].upper));

        traj.joint_names.push_back(m.type_joint);
        p.positions.push_back(target_rad);
      }
      
      if (!traj.joint_names.empty()){
        // p.time_from_start = rclcpp::Duration::from_seconds(0.1);
        traj.points.push_back(p);

        auto it = joint_pub.find(m.hand_joint_trajectory_topic);
        if (it != joint_pub.end() && it->second) {
          it->second->publish(traj);
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                              "Publisher for %s not found", m.hand_joint_trajectory_topic.c_str());
        }
      }
    }// Gripper
  }// Arm
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SOBITSTeleop>());
  rclcpp::shutdown();
  return 0;
}
