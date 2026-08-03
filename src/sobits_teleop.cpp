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
  // Action server name is configurable so this works for any robot exposing a
  // MoveToPose server under a different name; defaults to "move_to_pose".
  std::string pose_action_name = "move_to_pose";
  this->get_parameter("control_poses.pose_action", pose_action_name);
  move_to_pose_client = rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
      this, pose_action_name);
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
  if (teleop_rate_hz < 1.0 || teleop_rate_hz > 1000.0) {
    RCLCPP_WARN(get_logger(),
      "teleop_rate_hz=%.2f is out of sane bounds [1, 1000] — clamping", teleop_rate_hz);
    teleop_rate_hz = std::clamp(teleop_rate_hz, 1.0, 1000.0);
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

// Refuse to command a joint whose URDF limits aren't loaded yet.
bool SOBITSTeleop::clamp_to_limits_checked(const std::string & joint, double & value)
{
  auto it = joint_limits.find(joint);
  if (it == joint_limits.end()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "No URDF limits for '%s' yet — skipping command", joint.c_str());
    return false;
  }
  const auto [lo, hi] = std::minmax({it->second.lower, it->second.upper});
  value = std::clamp(value, lo, hi);
  return true;
}

void SOBITSTeleop::publish_arm_tracking(const std::string & arm, bool enabled)
{
  auto it = arm_track_pubs_.find(arm);
  if (it == arm_track_pubs_.end()) return;
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  it->second->publish(msg);
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

  // Load pose parameters. "trigger" is an optional modifier button held while
  // pressing the pose button; omit it to bind the pose button on its own.
  if (this->has_parameter("control_poses.pose_list")) {
    this->get_parameter("control_poses.pose_list", pose_list);
    for (const auto& pose_name : pose_list) {
      pm = PoseMap{};
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
  if (this->has_parameter("quest_control.groups")) {
    this->get_parameter("quest_control.groups", quest_groups);
    for (const auto& group : quest_groups) {
      if (group == "head") {
        this->get_parameter("quest_control." + group + ".vertical",        qhm.vertical);
        this->get_parameter("quest_control." + group + ".horizontal",      qhm.horizontal);
        this->get_parameter("quest_control." + group + ".enable_axis",     qhm.head_mode);
        this->get_parameter("quest_control." + group + ".vertical_sign",   qhm.vertical_sign);
        this->get_parameter("quest_control." + group + ".horizontal_sign", qhm.horizontal_sign);
        this->get_parameter("quest_control." + group + ".scale",           qhm.scale);
        this->get_parameter("robot_topic_name.joint_trajectory_topic." + group, qhm.head_joint_trajectory_topic);
        joint_pub[qhm.head_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          qhm.head_joint_trajectory_topic, 10);

        if (this->has_parameter("quest_control." + group + ".body_lift")) {
          this->get_parameter("quest_control." + group + ".body_lift",       qhm.body_lift);
          this->get_parameter("robot_topic_name.joint_trajectory_topic.body", qhm.body_joint_trajectory_topic);
          joint_pub[qhm.body_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            qhm.body_joint_trajectory_topic, 10);
        }
        continue;
      }

      // Group type is inferred from which fields are present.
      const bool is_arm  = this->has_parameter("quest_control." + group + ".end_effector_frame_name");
      const bool is_hand = this->has_parameter("quest_control." + group + ".pose_action") ||
                            this->has_parameter("quest_control." + group + ".adaptive.trigger_axis");

      if (is_arm) {
        QuestArmMap am{};
        am.group = group;
        this->get_parameter("quest_control." + group + ".controller",              am.controller);
        this->get_parameter("quest_control." + group + ".base_frame_name",         am.base_frame_name);
        this->get_parameter("quest_control." + group + ".end_effector_frame_name", am.end_effector_frame_name);
        this->get_parameter("quest_control." + group + ".target_frame_name",       am.target_frame_name);
        this->get_parameter("quest_control." + group + ".scale",                   am.scale);
        this->get_parameter("quest_control." + group + ".enable_axis",             am.enable_axis);
        this->get_parameter("robot_topic_name.joint_trajectory_topic." + group,    am.arm_joint_trajectory_topic);
        // Optional proximity thresholds — defaults are set in the struct
        if (this->has_parameter("quest_control." + group + ".proximity_threshold")) {
          this->get_parameter("quest_control." + group + ".proximity_threshold", am.proximity_threshold);
        }
        if (this->has_parameter("quest_control." + group + ".proximity_angle_threshold")) {
          this->get_parameter("quest_control." + group + ".proximity_angle_threshold", am.proximity_angle_threshold);
        }

        joint_pub[am.arm_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          am.arm_joint_trajectory_topic, 10);
        quest_arm_mappings[group] = am;
      } else if (is_hand) {
        QuestHandMap hm{};
        hm.group = group;
        this->get_parameter("quest_control." + group + ".controller", hm.controller);
        this->get_parameter("quest_control." + group + ".speed",      hm.speed);

        if (this->has_parameter("quest_control." + group + ".single_joint.axis")) {
          this->get_parameter("quest_control." + group + ".single_joint.axis", hm.type_axis);
          this->get_parameter("quest_control." + group + ".single_joint.name", hm.type_joint);
          if (this->has_parameter("quest_control." + group + ".single_joint.axis_sign"))
            this->get_parameter("quest_control." + group + ".single_joint.axis_sign", hm.type_sign);
          if (this->has_parameter("quest_control." + group + ".single_joint.min"))
            this->get_parameter("quest_control." + group + ".single_joint.min", hm.type_min);
          if (this->has_parameter("quest_control." + group + ".single_joint.max"))
            this->get_parameter("quest_control." + group + ".single_joint.max", hm.type_max);
        }
        if (this->has_parameter("quest_control." + group + ".pose_button")) {
          this->get_parameter("quest_control." + group + ".pose_button", hm.pose_button);
        }
        if (this->has_parameter("quest_control." + group + ".pose_open")) {
          this->get_parameter("quest_control." + group + ".pose_open",   hm.pose_open);
          this->get_parameter("quest_control." + group + ".pose_close",  hm.pose_close);
          this->get_parameter("quest_control." + group + ".pose_action", hm.pose_action);
        }
        if (this->has_parameter("quest_control." + group + ".adaptive.trigger_axis")) {
          this->get_parameter("quest_control." + group + ".adaptive.trigger_axis", hm.adaptive_trigger_axis);
          this->get_parameter("quest_control." + group + ".adaptive.stick_axis",   hm.adaptive_stick_axis);
          this->get_parameter("quest_control." + group + ".adaptive.axis_sign",    hm.adaptive_close_sign);

          // Load adaptive joint list: adaptive.names is a list of joint names,
          // each with close_pos, open_pos, and optional fixed flag.
          const std::string aj_prefix = "quest_control." + group + ".adaptive";
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
              hm.adaptive_joints.push_back(ajt);
            }
          }
        }
        quest_hand_mappings[group] = hm;
      } else {
        RCLCPP_WARN(get_logger(), "Quest group '%s' is neither arm nor hand — skipping", group.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu quest arm and %zu quest hand parameters from rosparam",
      quest_arm_mappings.size(), quest_hand_mappings.size());
    has_quest_controls = !quest_groups.empty();

    // Create one enable-publisher per arm (planning group)
    for (const auto & [arm_name, am] : quest_arm_mappings) {
      if (arm_track_pubs_.find(am.group) == arm_track_pubs_.end()) {
        // Reliable + transient_local (depth 1) so a late-starting subscriber
        // (e.g. the Servo target bridge, which may come up after this node)
        // still receives the current enable state instead of missing it.
        // moveit_arm_controller's volatile subscriber remains compatible: QoS
        // compatibility only requires publisher-durability >= subscriber-durability.
        arm_track_pubs_[am.group] = this->create_publisher<std_msgs::msg::Bool>(
          am.group + "/moveit_track_enabled",
          rclcpp::QoS(1).reliable().transient_local());
        RCLCPP_INFO(get_logger(),
          "Created arm track publisher for '%s'", am.group.c_str());
      }
    }
    // Create one hand pose action client per hand group
    for (const auto & [hand_name, hm] : quest_hand_mappings) {
      if (!hm.pose_action.empty() &&
          hand_pose_clients_.find(hm.group) == hand_pose_clients_.end()) {
        hand_pose_clients_[hm.group] =
          rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
            this, hm.pose_action);
        hand_open_state_[hm.group]  = true;
        hand_toggle_time_[hm.group] = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(get_logger(),
          "Created hand pose client for '%s' → '%s'",
          hm.group.c_str(), hm.pose_action.c_str());
      }
    }
  }

  requires_joint_states = !joint_mappings.empty() || has_quest_controls;

  if (requires_joint_states && joint_states_topic.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "joint_states_topic is required for joint or quest teleop, disabling those controls.");
    joint_mappings.clear();
    quest_arm_mappings.clear();
    quest_hand_mappings.clear();
    quest_groups.clear();
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
  // position can be shorter than name (velocity/effort-only publishers).
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; i++) {
    joint_pos[msg->name[i]] = msg->position[i];
  }
  if (n > 0) joint_state_initialized = true;
}

void SOBITSTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // previous_buttons is updated at the end of teleop() only, so a press edge
  // survives until the next tick even if several joy messages arrive between ticks.
  latest_axes      = msg->axes;
  latest_buttons   = msg->buttons;
  joy_received     = true;
}


void SOBITSTeleop::robot_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // Re-stamp EVERY transform with the local wall clock on arrival. Sources
  // stamp with their own clocks (robot: sim time ~300 s; Quest: its device
  // clock, observed up to ~4 min AHEAD of ours), and tf2 always serves the
  // newest stamp — so a skewed source's cached frames shadow live data long
  // after it disconnects. Arrival-time stamps make "newest" mean "most recently
  // received" and make the quest-frame staleness guard measure exactly whether
  // the stream is alive.
  const rclcpp::Time now_wall = wall_clock_->now();

  for (const auto & t : msg->transforms) {
    geometry_msgs::msg::TransformStamped ts = t;
    ts.header.stamp = now_wall;
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

    if (m.button < 0 || m.button >= static_cast<int>(latest_buttons.size())) continue;
    if (latest_buttons[m.button] == 0) continue;

    if (m.axis < 0 || m.axis >= static_cast<int>(latest_axes.size())) continue;
    float axis_val = latest_axes[m.axis];
    if (std::abs(axis_val) < 1e-3) continue;

    const bool fast = m.fast_button >= 0 &&
      m.fast_button < static_cast<int>(latest_buttons.size()) &&
      latest_buttons[m.fast_button] == 1;

    // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
    double delta_pos = axis_val * m.axis_sign * (fast ? m.fast_speed : m.speed) * jog_tick_scale_;
    double target = joint_pos[m.joint_name] + delta_pos;
    if (!clamp_to_limits_checked(m.joint_name, target)) continue;
    joint_pos[m.joint_name] = target;

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
    // A trigger of -1 means no modifier is required; otherwise it must be held.
    if (pose_map.trigger >= 0) {
      if (pose_map.trigger >= static_cast<int>(latest_buttons.size())) continue;
      if (latest_buttons[pose_map.trigger] == 0) continue;
    }

    bool button_just_pressed = pose_map.button >= 0 && 
                              pose_map.button < static_cast<int>(latest_buttons.size()) &&
                              latest_buttons[pose_map.button] == 1 &&
                              (previous_buttons.empty() || 
                               pose_map.button >= static_cast<int>(previous_buttons.size()) || 
                               previous_buttons[pose_map.button] == 0);
                               
    if (!button_just_pressed) continue;

    if (!move_to_pose_client->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "move_to_pose action server not ready — skipping pose '%s'", pose_map.pose_name.c_str());
      continue;
    }

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

  // Skip entirely if cmd_vel isn't configured — avoids flooding zero twists.
  if (cvm.button >= 0 || cvm.axis >= 0) {
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Twist stop;

    bool cmd_vel_enabled = false;
    bool fast_mode = false;

    if (cvm.button >= 0 &&
        cvm.button < static_cast<int>(latest_buttons.size())) {
      cmd_vel_enabled = (latest_buttons[cvm.button] == 1);
      if (cvm.fast_button >= 0 &&
          cvm.fast_button < static_cast<int>(latest_buttons.size())) {
        fast_mode = fast_mode || (latest_buttons[cvm.fast_button] == 1);
      }
    }
    if (cvm.axis >= 0 &&
        cvm.axis < static_cast<int>(latest_axes.size())) {
      // OR with the button branch — either enable source is allowed.
      cmd_vel_enabled = cmd_vel_enabled || (latest_axes[cvm.axis] > 0.5);
      if (cvm.fast_axis >= 0 &&
          cvm.fast_axis < static_cast<int>(latest_axes.size())) {
        fast_mode = fast_mode || (latest_axes[cvm.fast_axis] > 0.5);
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
    } else if (cmd_vel_was_enabled_) {
      // Publish stop once on the enabled->disabled edge, not every tick.
      cmd_vel_pub->publish(stop);
    }
    cmd_vel_was_enabled_ = cmd_vel_enabled;
  }

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
      // Reject frames whose stamp is far from the current wall clock, in either
      // direction. TimePointZero serves the newest cached transform forever, so
      // without this a disconnected headset's last pose (or future-stamped data
      // from a clock-skewed source) is indistinguishable from live input.
      const double age = std::abs(
        (wall_clock_->now() - rclcpp::Time(ts.header.stamp, RCL_SYSTEM_TIME)).seconds());
      if (age > kQuestTfMaxAgeSec) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *wall_clock_, 2000,
          "%s TF stamp is %.2f s from now (max %.2f) — treating as absent "
          "(stale cache or clock-skewed source)",
          quest_frame.c_str(), age, kQuestTfMaxAgeSec);
        return false;
      }
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

  if (this->has_parameter("quest_control.groups")) {
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

      // RCLCPP_INFO(this->get_logger(), "pub_joint_pos %.2f, %.2f", pan_target, tilt_target);
      if (clamp_to_limits_checked(qhm.horizontal, pan_target) &&
          clamp_to_limits_checked(qhm.vertical, tilt_target)) {
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
      }

      if (!qhm.body_lift.empty()) {
        dz = T_delta.getOrigin().z();
        body_lift_target = last_body_lift + qhm.scale * dz;
        if (clamp_to_limits_checked(qhm.body_lift, body_lift_target)) {
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

    // Lost or stale controller input unlatches its arm. Keeping a latch alive
    // on dead input freezes the target while the operator keeps moving, and the
    // next valid frame would then teleport it. Re-gripping (or input returning
    // while gripped) re-latches with a fresh zero-error capture, so recovery is
    // jump-free by construction.
    // Find the arm map for a given controller side ("right"/"left").
    auto find_arm = [&](const std::string & side) -> QuestArmMap * {
      for (auto & [name, m] : quest_arm_mappings) {
        if (m.controller == side) return &m;
      }
      return nullptr;
    };

    if (right_arm_latched && !right_tf_ok) {
      right_arm_latched = false;
      have_pub_prev_r_ = false;
      if (auto * m = find_arm("right")) publish_arm_tracking(m->group, false);
      RCLCPP_WARN(this->get_logger(), "Right controller TF stale/lost — right arm unlatched");
    }
    if (left_arm_latched && !left_tf_ok) {
      left_arm_latched = false;
      have_pub_prev_l_ = false;
      if (auto * m = find_arm("left")) publish_arm_tracking(m->group, false);
      RCLCPP_WARN(this->get_logger(), "Left controller TF stale/lost — left arm unlatched");
    }
    if (arm_tracking && !right_arm_latched && !left_arm_latched) {
      arm_tracking = false;
      RCLCPP_INFO(this->get_logger(), "Arm tracking stopped (controller input lost)");
    }

    // Determine arm_control_enabled from the first arm with a valid enable_axis
    // (both left and right share the same button in the default quest.yaml)
    for (auto &[name, m] : quest_arm_mappings) {
      if (m.enable_axis >= 0 && m.enable_axis < static_cast<int>(latest_axes.size())) {
        arm_control_enabled = (latest_axes[m.enable_axis] > 0.5);
        break;
      }
    }

    // Stop tracking immediately when the grip button is released.
    if (!arm_control_enabled && arm_tracking) {
      arm_tracking = false;
      right_arm_latched = false;
      left_arm_latched  = false;
      have_pub_prev_r_ = false;
      have_pub_prev_l_ = false;
      RCLCPP_INFO(this->get_logger(), "Arm tracking stopped");
      for (auto & [name, m] : quest_arm_mappings) {
        publish_arm_tracking(m.group, false);
      }
    }
    // Per-arm latching is handled inside the per-arm loop below, after the
    // fresh end-effector TF has been read and the proximity check can be done.

    for (auto &[name, m] : quest_arm_mappings) {
      if (m.controller == "right" && right_tf_ok && head_tf_ok) {
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

        // Compute raw target in base_footprint space (HMD + scaled controller delta from HMD).
        {
          tf2::Vector3 hmd_pos_odom = current_tf_hmd_odom.getOrigin();
          tf2::Vector3 delta_odom   = current_tf_r_odom.getOrigin() - hmd_pos_odom;
          T_target_r.setOrigin(hmd_pos_odom + delta_odom * m.scale);
          T_target_r.setRotation(current_tf_r_odom.getRotation());
        }

        // Per-arm latch: requires grip button. Proximity check is skipped when both
        // thresholds are 0 (immediate latch). On latch the raw target and EE pose are
        // captured; the published target is re-zeroed onto the EE so tracking starts
        // without a jump.
        if (arm_control_enabled && !right_arm_latched && ee_r_ok) {
          bool prox_ok = true;
          if (m.proximity_threshold > 0.0 || m.proximity_angle_threshold > 0.0) {
            tf2::Vector3 pos_diff = current_tf_r.getOrigin() - current_tf_ee_r.getOrigin();
            double pos_err = pos_diff.length();
            tf2::Quaternion q_diff =
              current_tf_ee_r.getRotation().inverse() * current_tf_r.getRotation();
            q_diff.normalize();
            double angle_err = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

            if (m.proximity_threshold > 0.0 && pos_err > m.proximity_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Right arm: controller %.3f m from EE (threshold %.3f m) — move controller to EE before gripping",
                pos_err, m.proximity_threshold);
              prox_ok = false;
            }
            if (prox_ok && m.proximity_angle_threshold > 0.0 &&
                angle_err > m.proximity_angle_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Right arm: controller %.1f deg from EE orientation (threshold %.1f deg) — align controller before gripping",
                angle_err * 180.0 / M_PI,
                m.proximity_angle_threshold * 180.0 / M_PI);
              prox_ok = false;
            }
          }
            if (prox_ok) {
            right_arm_latched = true;
            right_arm_just_latched = true;
            T_ctrl_latch_r = current_tf_r_odom;
            T_ee_latch_r   = current_tf_ee_r;
            latch_time_r_  = this->now();
            have_pub_prev_r_ = false;
          }
        }

        // Re-zero onto the EE while latched: the published target tracks EE_latch
        // composed with the CONTROLLER's motion delta since the latch instant.
        // Deltas come from the controller pose alone — not the HMD-anchored raw
        // mapping (scale*ctrl + (1-scale)*hmd), which leaks head sway into the
        // target whenever scale != 1 and makes the arm drift with the controller
        // at rest. A soft-start ramps the delta in after latch so the physical
        // jerk of squeezing the grip does not become a sudden arm motion.
        tf2::Transform T_pub_r = T_target_r;
        if (right_arm_latched) {
          const double ramp = std::clamp(
            (this->now() - latch_time_r_).seconds() / kLatchSoftStartSec, 0.0, 1.0);
          const tf2::Vector3 dpos =
            (current_tf_r_odom.getOrigin() - T_ctrl_latch_r.getOrigin()) *
            static_cast<double>(m.scale) * ramp;
          T_pub_r.setOrigin(T_ee_latch_r.getOrigin() + dpos);
          tf2::Quaternion q_delta_r =
            current_tf_r_odom.getRotation() * T_ctrl_latch_r.getRotation().inverse();
          q_delta_r = tf2::Quaternion::getIdentity().slerp(q_delta_r.normalized(), ramp);
          T_pub_r.setRotation((q_delta_r * T_ee_latch_r.getRotation()).normalized());

          // Safety net: rate-limit the published target's motion. Whatever goes
          // wrong upstream (mixed TF sources, stale caches, math bugs), the
          // target can only crawl away from the arm, never jump.
          if (have_pub_prev_r_) {
            const double dt_s = 1.0 / teleop_rate_hz;
            const double max_lin = kMaxTargetLinVel * dt_s;
            tf2::Vector3 dp = T_pub_r.getOrigin() - T_pub_prev_r_.getOrigin();
            const double d = dp.length();
            if (d > max_lin) {
              T_pub_r.setOrigin(T_pub_prev_r_.getOrigin() + dp * (max_lin / d));
            }
            const double max_ang = kMaxTargetAngVel * dt_s;
            tf2::Quaternion q_step =
              T_pub_r.getRotation() * T_pub_prev_r_.getRotation().inverse();
            const double ang = q_step.normalized().getAngleShortestPath();
            if (ang > max_ang) {
              T_pub_r.setRotation(T_pub_prev_r_.getRotation()
                .slerp(T_pub_r.getRotation(), max_ang / ang).normalized());
            }
          }
          T_pub_prev_r_ = T_pub_r;
          have_pub_prev_r_ = true;
        }

        // Publish target under base_footprint (visualization — stays fixed in robot space).
        target_msg_r.header.stamp    = this->now();
        target_msg_r.header.frame_id = "base_footprint";
        target_msg_r.child_frame_id  = m.target_frame_name;
        target_msg_r.transform       = tf2::toMsg(T_pub_r);
        tf_broadcaster->sendTransform(target_msg_r);

        // Enable tracking only AFTER the first re-zeroed target is on TF: if the
        // enable were published inside the latch block (before sendTransform), a
        // backend waking in between would plan toward the stale pre-latch raw
        // target — exactly the startup jump the re-zeroing exists to prevent.
        if (right_arm_just_latched) {
          right_arm_just_latched = false;
          const char * prox_note = (m.proximity_threshold <= 0.0 &&
                                    m.proximity_angle_threshold <= 0.0)
                                   ? " (calibration skipped)" : "";
          // Publish on every latch, not just the first (re-latch after unlatch).
          publish_arm_tracking(m.group, true);
          if (!arm_tracking) {
            arm_tracking = true;
            RCLCPP_INFO(this->get_logger(), "Arm tracking started (right latched%s)", prox_note);
          } else {
            RCLCPP_INFO(this->get_logger(), "Right arm latched%s", prox_note);
          }
        }
      }

      if (m.controller == "left" && left_tf_ok && head_tf_ok) {
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

        // Compute raw target in base_footprint space.
        {
          tf2::Vector3 hmd_pos_odom = current_tf_hmd_odom.getOrigin();
          tf2::Vector3 delta_odom   = current_tf_l_odom.getOrigin() - hmd_pos_odom;
          T_target_l.setOrigin(hmd_pos_odom + delta_odom * m.scale);
          T_target_l.setRotation(current_tf_l_odom.getRotation());
        }

        // Per-arm latch: requires grip button. Proximity check is skipped when both
        // thresholds are 0 (immediate latch). On latch the raw target and EE pose are
        // captured; the published target is re-zeroed onto the EE so tracking starts
        // without a jump.
        if (arm_control_enabled && !left_arm_latched && ee_l_ok) {
          bool prox_ok = true;
          if (m.proximity_threshold > 0.0 || m.proximity_angle_threshold > 0.0) {
            tf2::Vector3 pos_diff = current_tf_l.getOrigin() - current_tf_ee_l.getOrigin();
            double pos_err = pos_diff.length();
            tf2::Quaternion q_diff =
              current_tf_ee_l.getRotation().inverse() * current_tf_l.getRotation();
            q_diff.normalize();
            double angle_err = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

            if (m.proximity_threshold > 0.0 && pos_err > m.proximity_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Left arm: controller %.3f m from EE (threshold %.3f m) — move controller to EE before gripping",
                pos_err, m.proximity_threshold);
              prox_ok = false;
            }
            if (prox_ok && m.proximity_angle_threshold > 0.0 &&
                angle_err > m.proximity_angle_threshold) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Left arm: controller %.1f deg from EE orientation (threshold %.1f deg) — align controller before gripping",
                angle_err * 180.0 / M_PI,
                m.proximity_angle_threshold * 180.0 / M_PI);
              prox_ok = false;
            }
          }
          if (prox_ok) {
            left_arm_latched = true;
            left_arm_just_latched = true;
            T_ctrl_latch_l = current_tf_l_odom;
            T_ee_latch_l   = current_tf_ee_l;
            latch_time_l_  = this->now();
            have_pub_prev_l_ = false;
          }
        }

        // Re-zero onto the EE while latched: controller-only deltas with a
        // soft-start ramp (see the matching comment in the right-arm block).
        tf2::Transform T_pub_l = T_target_l;
        if (left_arm_latched) {
          const double ramp = std::clamp(
            (this->now() - latch_time_l_).seconds() / kLatchSoftStartSec, 0.0, 1.0);
          const tf2::Vector3 dpos =
            (current_tf_l_odom.getOrigin() - T_ctrl_latch_l.getOrigin()) *
            static_cast<double>(m.scale) * ramp;
          T_pub_l.setOrigin(T_ee_latch_l.getOrigin() + dpos);
          tf2::Quaternion q_delta_l =
            current_tf_l_odom.getRotation() * T_ctrl_latch_l.getRotation().inverse();
          q_delta_l = tf2::Quaternion::getIdentity().slerp(q_delta_l.normalized(), ramp);
          T_pub_l.setRotation((q_delta_l * T_ee_latch_l.getRotation()).normalized());

          // Safety net: rate-limit the published target's motion (see the
          // matching comment in the right-arm block).
          if (have_pub_prev_l_) {
            const double dt_s = 1.0 / teleop_rate_hz;
            const double max_lin = kMaxTargetLinVel * dt_s;
            tf2::Vector3 dp = T_pub_l.getOrigin() - T_pub_prev_l_.getOrigin();
            const double d = dp.length();
            if (d > max_lin) {
              T_pub_l.setOrigin(T_pub_prev_l_.getOrigin() + dp * (max_lin / d));
            }
            const double max_ang = kMaxTargetAngVel * dt_s;
            tf2::Quaternion q_step =
              T_pub_l.getRotation() * T_pub_prev_l_.getRotation().inverse();
            const double ang = q_step.normalized().getAngleShortestPath();
            if (ang > max_ang) {
              T_pub_l.setRotation(T_pub_prev_l_.getRotation()
                .slerp(T_pub_l.getRotation(), max_ang / ang).normalized());
            }
          }
          T_pub_prev_l_ = T_pub_l;
          have_pub_prev_l_ = true;
        }

        // Publish target under base_footprint (visualization).
        target_msg_l.header.stamp    = this->now();
        target_msg_l.header.frame_id = "base_footprint";
        target_msg_l.child_frame_id  = m.target_frame_name;
        target_msg_l.transform       = tf2::toMsg(T_pub_l);
        tf_broadcaster->sendTransform(target_msg_l);

        // Enable tracking only AFTER the first re-zeroed target is on TF (see the
        // matching comment in the right-arm block).
        if (left_arm_just_latched) {
          left_arm_just_latched = false;
          const char * prox_note = (m.proximity_threshold <= 0.0 &&
                                    m.proximity_angle_threshold <= 0.0)
                                   ? " (calibration skipped)" : "";
          // Publish on every latch, not just the first (see right-arm block).
          publish_arm_tracking(m.group, true);
          if (!arm_tracking) {
            arm_tracking = true;
            RCLCPP_INFO(this->get_logger(), "Arm tracking started (left latched%s)", prox_note);
          } else {
            RCLCPP_INFO(this->get_logger(), "Left arm latched%s", prox_note);
          }
        }
      }

    }// Arm

    // Gripper — separate loop over hand groups, runs after the arm loop.
    for (auto &[name, m] : quest_hand_mappings) {
        // ── 1. Hand pose toggle (open / close) on button press ───────────────
        // Configured via pose_button / pose_open / pose_close / pose_action in quest.yaml.
        auto hp_client_it = hand_pose_clients_.find(name);
        if (m.pose_button >= 0 && hp_client_it != hand_pose_clients_.end()) {
          rclcpp::Time & toggle_time = hand_toggle_time_.at(name);
          const bool debounce_ok = (this->now() - toggle_time).seconds() > 0.4;

          if (m.pose_button < static_cast<int>(latest_buttons.size()) &&
              latest_buttons[m.pose_button] == 1 &&
              (previous_buttons.empty() ||
               m.pose_button >= static_cast<int>(previous_buttons.size()) ||
               previous_buttons[m.pose_button] == 0) &&
              debounce_ok)
          {
            auto & client = hp_client_it->second;
            // Check server readiness before flipping state, non-blocking.
            if (client->action_server_is_ready()) {
              toggle_time = this->now();
              bool & is_open = hand_open_state_.at(name);
              is_open = !is_open;
              const std::string pose_name = is_open ? m.pose_open : m.pose_close;

              auto goal = sobits_interfaces::action::MoveToPose::Goal();
              goal.pose_name = pose_name;
              // joint_action_server uses time_allowance as the trajectory's
              // time_from_start, so this IS the open/close motion duration —
              // 5 s made the gripper crawl. 1 s is ample for the hand joints.
              goal.time_allowance.sec = 1;
              // Suppress this hand's adaptive goal stream until the pose motion
              // completes, so per-tick "hold" goals don't fight the trajectory.
              m.hand_pose_in_flight = true;
              m.hand_pose_deadline = this->now() + rclcpp::Duration::from_seconds(2.0);
              bool * pose_in_flight = &m.hand_pose_in_flight;
              auto opts = rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SendGoalOptions();
              opts.goal_response_callback =
                [pose_in_flight](rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveToPose>::SharedPtr h) {
                  if (!h) *pose_in_flight = false;  // rejected: resume adaptive
                };
              opts.result_callback = [this, pose_name, pose_in_flight](const auto & result) {
                *pose_in_flight = false;
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
        // Configured via adaptive.trigger_axis / stick_axis / axis_sign / names in quest.yaml.
        // adaptive_close_sign: +1 → positive stick = close, -1 → negative = close.
        // A pose toggle that is still executing takes precedence over the
        // adaptive stream for this hand (both drive the same hand controller).
        if (m.hand_pose_in_flight && this->now() >= m.hand_pose_deadline) {
          m.hand_pose_in_flight = false;
        }
        if (m.adaptive_trigger_axis >= 0 && !m.adaptive_joints.empty() &&
            !m.hand_pose_in_flight &&
            m.adaptive_trigger_axis < static_cast<int>(latest_axes.size()) &&
            m.adaptive_stick_axis   < static_cast<int>(latest_axes.size()) &&
            latest_axes[m.adaptive_trigger_axis] > 0.1)
        {
          float raw_stick  = latest_axes[m.adaptive_stick_axis];
          float close_frac = std::clamp( raw_stick * m.adaptive_close_sign, 0.0f, 1.0f);
          float open_frac  = std::clamp(-raw_stick * m.adaptive_close_sign, 0.0f, 1.0f);
          float deflection = std::max(close_frac, open_frac);

          // Vertical stick (single_joint.axis) rotates single_joint.name — the
          // grip-configuration knuckle — while the adaptive trigger is held.
          // Small deadzone only to reject stick noise; past it the deflection is
          // rescaled so the step ramps up from zero instead of jumping straight to
          // half speed. A large deadzone here makes the jog feel stepwise rather
          // than continuous, and lets tremor near the threshold toggle it on/off.
          constexpr float kVjogDeadzone = 0.15f;
          float vjog_stick = 0.0f;
          if (m.type_axis >= 0 && m.type_axis < static_cast<int>(latest_axes.size())) {
            const float raw = latest_axes[m.type_axis];
            if (std::abs(raw) > kVjogDeadzone) {
              vjog_stick = std::copysign(
                (std::abs(raw) - kVjogDeadzone) / (1.0f - kVjogDeadzone), raw);
            }
          }

          // Endpoint-swing mode, active when a functional range is configured
          // (single_joint.min/max): the grip-type knuckle works against a spring,
          // and the current-capped servo cannot break its stiction with small
          // incremental position errors — a distant target sustains full torque
          // and demonstrably moves it. The knuckle is a binary 2f/3f switch in
          // practice, so one stick flick = one full swing to the flicked
          // endpoint; the stick must return to center to re-arm. Configs without
          // a range keep the legacy incremental jog.
          const bool vjog_has_range = (m.type_min > -1e8f && m.type_max < 1e8f);
          // The flick fires only on a DECISIVE, DOMINANTLY-vertical push. The
          // open/close control shares the same physical stick (left/right), and
          // with only the small jog deadzone as the trigger, a firm horizontal
          // push with a slight vertical component fired the 2f/3f swing and
          // changed the finger pose mid-grasp. Re-arm keeps the low deadzone,
          // giving hysteresis between fire (0.6) and re-arm (0.15).
          constexpr float kFlickThreshold = 0.6f;
          float vjog_h = 0.0f;
          if (m.adaptive_stick_axis >= 0 &&
              m.adaptive_stick_axis < static_cast<int>(latest_axes.size())) {
            vjog_h = latest_axes[m.adaptive_stick_axis];
          }
          const float vjog_raw =
            (m.type_axis >= 0 && m.type_axis < static_cast<int>(latest_axes.size()))
              ? latest_axes[m.type_axis] : 0.0f;
          const bool vjog_decisive = std::abs(vjog_raw) > kFlickThreshold &&
                                     std::abs(vjog_raw) > std::abs(vjog_h);
          const bool vjog_fire = vjog_has_range && vjog_decisive && m.vjog_armed;
          if (vjog_has_range && vjog_stick == 0.0f) m.vjog_armed = true;
          // Keep goals flowing while a swing is in progress even with the stick
          // released — the endpoint command must persist until arrival.
          const bool vjog_request =
            vjog_has_range ? (vjog_fire || m.vjog_swing_active) : (vjog_stick != 0.0f);

          if (deflection >= 0.1f || vjog_request) {
            const bool closing = (close_frac >= open_frac);
            // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
            float step = m.speed * deflection * static_cast<float>(jog_tick_scale_);
            float vjog_step = m.speed * vjog_stick * static_cast<float>(m.type_sign) * static_cast<float>(jog_tick_scale_);

            // A flick starts (or redirects) a persistent endpoint swing: every
            // goal until arrival commands the knuckle to the endpoint, so the
            // large sustained position error the gear spring demands persists
            // across ordinary loop-rate goals — open/close rides in the same
            // goals instead of being blocked behind one long swing goal.
            if (vjog_fire) {
              m.vjog_armed = false;
              m.vjog_swing_active = true;
              m.vjog_swing_target = (vjog_step > 0.0f) ? m.type_max : m.type_min;
              m.vjog_swing_deadline = this->now() + rclcpp::Duration::from_seconds(2.5);
            }
            if (m.vjog_swing_active) {
              // Only check arrival if the joint is known; else rely on the deadline.
              const bool arrived = joint_pos.count(m.type_joint) &&
                std::abs(static_cast<float>(joint_pos.at(m.type_joint)) - m.vjog_swing_target) < 0.08f;
              if (arrived || this->now() >= m.vjog_swing_deadline) {
                m.vjog_swing_active = false;
              }
            }

            auto clamp_to_limits = [&](const std::string & jname, float value) -> float {
              if (!joint_limits.count(jname)) return value;
              return std::clamp(value,
                static_cast<float>(std::min(joint_limits.at(jname).lower, joint_limits.at(jname).upper)),
                static_cast<float>(std::max(joint_limits.at(jname).lower, joint_limits.at(jname).upper)));
            };

            auto step_toward = [&](const std::string & jname, float target) -> float {
              if (joint_pos.find(jname) == joint_pos.end()) return target;
              float cur = static_cast<float>(joint_pos.at(jname));
              float dir = (target > cur) ? 1.0f : -1.0f;
              float next = cur + dir * step;
              if ((dir > 0 && next > target) || (dir < 0 && next < target)) next = target;
              return clamp_to_limits(jname, next);
            };

            auto goal = sobits_interfaces::action::MoveJoint::Goal();
            float max_delta = 0.f;
            for (const auto & ajt : m.adaptive_joints) {
              goal.target_joint_names.push_back(ajt.name);
              float cur = joint_pos.count(ajt.name)
                ? static_cast<float>(joint_pos.at(ajt.name)) : ajt.close_pos;
              float tgt;
              if (ajt.fixed) {
                // Fixed joints HOLD their current position (the pose buttons set
                // the base grip configuration) — except the grip-type knuckle,
                // which the vertical stick rotates freely.
                tgt = cur;
                if (ajt.name == m.type_joint) {
                  if (m.vjog_swing_active) {
                    // Swing in progress: command the endpoint (large sustained
                    // error) in every goal until the knuckle arrives.
                    tgt = clamp_to_limits(ajt.name, m.vjog_swing_target);
                  } else if (!vjog_has_range && vjog_stick != 0.0f) {
                    tgt = clamp_to_limits(ajt.name, cur + vjog_step);
                  }
                }
                goal.target_joint_rad.push_back(tgt);
              } else if (deflection >= 0.1f) {
                tgt = closing ? ajt.close_pos : ajt.open_pos;
                tgt = step_toward(ajt.name, tgt);
                goal.target_joint_rad.push_back(tgt);
              } else {
                tgt = cur;
                goal.target_joint_rad.push_back(cur);  // vertical-only: hold curl
              }
              max_delta = std::max(max_delta, std::abs(tgt - cur));
            }
            // Scale duration to the largest excursion so a swing isn't ballistic.
            constexpr double kMaxHandJointVel = 3.0;  // rad/s — hand servo jog ceiling
            const double goal_sec = std::max(1.0 / teleop_rate_hz, max_delta / kMaxHandJointVel);
            goal.time_allowance = rclcpp::Duration::from_seconds(goal_sec);

            // One jog goal in flight PER HAND. The server accepts every goal and
            // runs it on its own detached thread without preempting the previous
            // one, so flooding stacks up overlapping threads publishing stale
            // targets — but a single global gate made the two hands block each
            // other. The mapping storage is stable after startup, so capturing a
            // pointer to the per-hand flag in the callbacks is safe.
            // Deadline backstops a lost result so the flag can't stick true forever.
            if (move_joint_client->action_server_is_ready() &&
                (!m.jog_goal_in_flight || this->now() >= m.jog_goal_deadline)) {
              m.jog_goal_in_flight = true;
              // Backstop must outlive the goal's own allowance.
              m.jog_goal_deadline = this->now() +
                rclcpp::Duration::from_seconds(std::max(1.0, goal_sec + 0.5));
              bool * in_flight = &m.jog_goal_in_flight;
              auto opts = rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SendGoalOptions();
              opts.goal_response_callback =
                [in_flight](rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveJoint>::SharedPtr h) {
                  if (!h) *in_flight = false;  // rejected: free the slot
                };
              opts.result_callback = [in_flight](const auto &) { *in_flight = false; };
              move_joint_client->async_send_goal(goal, opts);
            }
          }
        }
    }// Gripper
  }// Quest controllers

  // Consume edges once per tick (teleop() runs faster than joy publishes).
  previous_buttons = latest_buttons;
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::SOBITSTeleop)
