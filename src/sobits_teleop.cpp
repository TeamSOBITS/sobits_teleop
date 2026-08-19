#include "sobits_teleop/sobits_teleop.hpp"

namespace sobits_teleop
{

// Axis value above which a hold-style axis (trigger/mode switch) reads as "on".
constexpr double kAxisHoldThreshold = 0.5;

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
  get_param("control_poses.pose_action", pose_action_name);
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
  async_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this,
      robot_description_source_node);

  urdf_timer = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&SOBITSTeleop::load_joint_limits, this));

  load_parameters();

  if (!this->has_parameter("teleop_rate_hz")) {
    this->declare_parameter("teleop_rate_hz", teleop_rate_hz);
  }
  this->get_parameter("teleop_rate_hz", teleop_rate_hz);
  if (teleop_rate_hz < 1.0 || teleop_rate_hz > 1000.0) {
    RCLCPP_WARN(get_logger(),
      "teleop_rate_hz=%.2f is out of sane bounds [1, 1000] — clamping", teleop_rate_hz);
    teleop_rate_hz = std::clamp(teleop_rate_hz, 1.0, 1000.0);
  }
  // Config speeds are radians per legacy 50 ms tick; scale so teleop_rate_hz doesn't change jog speed.
  jog_tick_scale_ = (1.0 / teleop_rate_hz) / 0.05;

  timer = create_wall_timer(
    std::chrono::duration<double>(1.0 / teleop_rate_hz),
    std::bind(&SOBITSTeleop::teleop, this));
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void SOBITSTeleop::load_joint_limits()
{
  if (!requires_joint_states) {return;}
  if (urdf_loaded) {return;}

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

  if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {return;}

  const std::string urdf_xml = params[0].as_string();
  if (urdf_xml.empty()) {return;}

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

    if (!joint) {continue;}

    if (joint->limits) {
      Limit lim;
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
  if (it == arm_track_pubs_.end()) {return;}
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  it->second->publish(msg);
}

// True if any arm's own enable axis is currently held (drives the shared release path).
bool SOBITSTeleop::any_arm_enable_held()
{
  for (const auto & [name, m] : quest_arm_mappings) {
    if (axis_held(m.enable_axis)) {
      return true;
    }
  }
  return false;
}

// Reports which optional config blocks this device.yaml supplies, so a block
// missing on one robot is visible at startup instead of silently doing nothing.
// Endpoint is either a pose name already in control_poses, or joints and
// positions written under the blend itself.
bool SOBITSTeleop::resolve_blend_endpoint(
  const std::string & base, const std::string & key, const std::string & group,
  BlendEndpoint & out)
{
  get_param(base + "." + key, out.pose_name);

  if (!out.pose_name.empty()) {
    for (const auto & pm : pose_mappings) {
      if (pm.pose_name != out.pose_name) {continue;}
      for (const auto & pg : pm.joint_groups) {
        if (!group.empty() && pg.joint_trajectory_topic != group_trajectory_topic(group)) {
          continue;
        }
        out.joint_names = pg.joint_names;
        out.positions = pg.positions;
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(),
      "Blend endpoint '%s' names pose '%s', which no control_poses entry defines "
      "for group '%s'", key.c_str(), out.pose_name.c_str(), group.c_str());
    return false;
  }

  get_param(base + "." + key + "_joints", out.joint_names);
  get_param(base + "." + key + "_positions", out.positions);
  return !out.joint_names.empty();
}

std::string SOBITSTeleop::group_trajectory_topic(const std::string & group)
{
  std::string topic;
  get_param("robot_topic_name.joint_trajectory_topic." + group, topic);
  if (topic.empty()) {
    RCLCPP_ERROR(get_logger(),
      "Group '%s' has no robot_topic_name.joint_trajectory_topic entry — skipping",
      group.c_str());
  }
  return topic;
}

void SOBITSTeleop::report_config_summary()
{
  const std::pair<const char *, bool> blocks[] = {
    {"control_joints", !joint_mappings.empty()},
    {"control_poses", !pose_mappings.empty()},
    {"control_velocity", cvm.button >= 0 || cvm.axis >= 0},
    {"quest_control", has_quest_controls},
  };

  std::string configured, absent;
  for (const auto & [name, present] : blocks) {
    std::string & dst = present ? configured : absent;
    if (!dst.empty()) {dst += ", ";}
    dst += name;
  }

  RCLCPP_INFO(get_logger(), "Config: %s",
    configured.empty() ? "no control blocks configured" : configured.c_str());
  if (!absent.empty()) {
    RCLCPP_INFO(get_logger(), "Config: not configured — %s", absent.c_str());
  }
}

bool SOBITSTeleop::button_down(int idx) const
{
  return idx >= 0 && idx < static_cast<int>(latest_buttons.size()) && latest_buttons[idx] == 1;
}

bool SOBITSTeleop::button_pressed(int idx) const
{
  return button_down(idx) &&
         (previous_buttons.empty() ||
         idx >= static_cast<int>(previous_buttons.size()) ||
         previous_buttons[idx] == 0);
}

bool SOBITSTeleop::axis_held(int idx) const
{
  return idx >= 0 && idx < static_cast<int>(latest_axes.size()) &&
         latest_axes[idx] > kAxisHoldThreshold;
}

double SOBITSTeleop::axis_value(int idx) const
{
  return (idx >= 0 && idx < static_cast<int>(latest_axes.size())) ? latest_axes[idx] : 0.0;
}

// EE lookup, latch, soft-start ramp, rate limiter, and target broadcast for one arm.
void SOBITSTeleop::process_arm(
  const QuestArmMap & m, ArmTrackState & st, bool /*head_tf_ok*/,
  bool arm_enabled)
{
  bool ee_ok = false;
  try {
    geometry_msgs::msg::TransformStamped ee_msg = tf_buffer->lookupTransform(
      base_frame,
      m.end_effector_frame_name,
        tf2::TimePointZero,
        tf2::Duration(0)
    );
    tf2::fromMsg(ee_msg.transform, st.current_tf_ee);
    ee_ok = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
      "%s EE TF lookup failed: %s", m.controller.c_str(), ex.what());
  }

  // Compute raw target in base_footprint space (HMD + scaled controller delta from HMD).
  tf2::Transform T_target;
  {
    tf2::Vector3 hmd_pos_odom = current_tf_hmd_odom.getOrigin();
    tf2::Vector3 delta_odom = st.current_tf_odom.getOrigin() - hmd_pos_odom;
    T_target.setOrigin(hmd_pos_odom + delta_odom * m.motion_scale);
    T_target.setRotation(st.current_tf_odom.getRotation());
  }

  // Releasing this arm's own grip unlatches it, independent of the other arm.
  if (!arm_enabled && st.latched) {
    st.latched = false;
    st.have_pub_prev = false;
    publish_arm_tracking(m.group, false);
    RCLCPP_INFO(this->get_logger(), "%s arm unlatched (grip released)", m.controller.c_str());
  }

  // Per-arm latch on grip: proximity check skipped when thresholds are 0;
  // the target is re-zeroed onto the EE so tracking starts jump-free.
  if (arm_enabled && !st.latched && ee_ok) {
    bool prox_ok = true;
    if (m.proximity_threshold > 0.0 || m.proximity_angle_threshold > 0.0) {
      tf2::Vector3 pos_diff = st.current_tf.getOrigin() - st.current_tf_ee.getOrigin();
      double pos_err = pos_diff.length();
      tf2::Quaternion q_diff =
        st.current_tf_ee.getRotation().inverse() * st.current_tf.getRotation();
      q_diff.normalize();
      double angle_err = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

      if (m.proximity_threshold > 0.0 && pos_err > m.proximity_threshold) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
          "%s arm: controller %.3f m from EE (threshold %.3f m) — move controller to EE before gripping",
          m.controller.c_str(), pos_err, m.proximity_threshold);
        prox_ok = false;
      }
      if (prox_ok && m.proximity_angle_threshold > 0.0 &&
        angle_err > m.proximity_angle_threshold)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
          "%s arm: controller %.1f deg from EE orientation (threshold %.1f deg) — align controller before gripping",
          m.controller.c_str(),
          angle_err * 180.0 / M_PI,
          m.proximity_angle_threshold * 180.0 / M_PI);
        prox_ok = false;
      }
    }
    if (prox_ok) {
      st.latched = true;
      st.just_latched = true;
      st.T_ctrl_latch = st.current_tf_odom;
      st.T_ee_latch = st.current_tf_ee;
      st.latch_time = this->now();
      st.have_pub_prev = false;
    }
  }

  // While latched: target = EE_latch + controller-only delta (the HMD term would leak
  // head sway); a soft-start ramp keeps the grip-squeeze jerk from moving the arm.
  tf2::Transform T_pub = T_target;
  if (st.latched) {
    const double ramp = std::clamp(
      (this->now() - st.latch_time).seconds() / kLatchSoftStartSec, 0.0, 1.0);
    const tf2::Vector3 dpos =
      (st.current_tf_odom.getOrigin() - st.T_ctrl_latch.getOrigin()) *
      static_cast<double>(m.motion_scale) * ramp;
    T_pub.setOrigin(st.T_ee_latch.getOrigin() + dpos);
    tf2::Quaternion q_delta =
      st.current_tf_odom.getRotation() * st.T_ctrl_latch.getRotation().inverse();
    q_delta = tf2::Quaternion::getIdentity().slerp(q_delta.normalized(), ramp);
    T_pub.setRotation((q_delta * st.T_ee_latch.getRotation()).normalized());

    // Safety net: rate-limit target motion so upstream faults can only crawl, never jump.
    if (st.have_pub_prev) {
      const double dt_s = tick_period();
      const double max_lin = kMaxTargetLinVel * dt_s;
      tf2::Vector3 dp = T_pub.getOrigin() - st.T_pub_prev.getOrigin();
      const double d = dp.length();
      if (d > max_lin) {
        T_pub.setOrigin(st.T_pub_prev.getOrigin() + dp * (max_lin / d));
      }
      const double max_ang = kMaxTargetAngVel * dt_s;
      tf2::Quaternion q_step =
        T_pub.getRotation() * st.T_pub_prev.getRotation().inverse();
      const double ang = q_step.normalized().getAngleShortestPath();
      if (ang > max_ang) {
        T_pub.setRotation(st.T_pub_prev.getRotation()
          .slerp(T_pub.getRotation(), max_ang / ang).normalized());
      }
    }
    st.T_pub_prev = T_pub;
    st.have_pub_prev = true;
  }

  // Publish target under base_footprint (visualization — stays fixed in robot space).
  geometry_msgs::msg::TransformStamped target_msg;
  target_msg.header.stamp = this->now();
  target_msg.header.frame_id = base_frame;
  target_msg.child_frame_id = m.target_frame_name;
  target_msg.transform = tf2::toMsg(T_pub);
  tf_broadcaster->sendTransform(target_msg);

  // Enable tracking only after the first re-zeroed target is on TF, else a
  // waking backend would plan toward the stale pre-latch target.
  if (st.just_latched) {
    st.just_latched = false;
    const char * prox_note = (m.proximity_threshold <= 0.0 &&
      m.proximity_angle_threshold <= 0.0) ?
      " (calibration skipped)" : "";
    // Publish on every latch, not just the first (re-latch after unlatch).
    publish_arm_tracking(m.group, true);
    if (!arm_tracking) {
      arm_tracking = true;
      RCLCPP_INFO(this->get_logger(), "Arm tracking started (%s latched%s)",
        m.controller.c_str(), prox_note);
    } else {
      RCLCPP_INFO(this->get_logger(), "%s arm latched%s", m.controller.c_str(), prox_note);
    }
  }
}

void SOBITSTeleop::load_parameters()
{
  get_param("robot_topic_name.joint_states_topic", joint_states_topic);
  get_param("robot_topic_name.base_frame", base_frame);

  // robot.yaml lists every group's topic; a device.yaml only uses a subset.
  mark_visited("robot_topic_name.joint_trajectory_topic");

  get_param("robot_topic_name.cmd_vel_topic", cvm.topic);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    cvm.topic, 10);

  // Load joint parameters
  if (has_param("control_joints.groups")) {
    get_param("control_joints.groups", joint_groups);

    for (const auto & joint_group : joint_groups) {
      if (!get_param("control_joints." + joint_group + ".joints_name", joint_names)) {
        mark_visited("control_joints." + joint_group);
        continue;
      }

      for (const auto & joint_name : joint_names) {
        JointMap jm;
        jm.joint_group = joint_group;
        jm.joint_name = joint_name;
        get_param("control_joints." + joint_group + "." + joint_name + ".button",
            jm.button);
        get_param("control_joints." + joint_group + "." + joint_name + ".enable_axis",
            jm.enable_axis);
        get_param("control_joints." + joint_group + "." + joint_name + ".fast_button",
            jm.fast_button);
        get_param("control_joints." + joint_group + "." + joint_name + ".fast_axis",
            jm.fast_axis);
        get_param("control_joints." + joint_group + "." + joint_name + ".axis", jm.axis);
        get_param("control_joints." + joint_group + "." + joint_name + ".axis_sign",
            jm.axis_sign);
        get_param("control_joints." + joint_group + "." + joint_name + ".speed",
            jm.speed);
        get_param("control_joints." + joint_group + "." + joint_name + ".fast_speed",
            jm.fast_speed);
        jm.joint_trajectory_topic = group_trajectory_topic(joint_group);
        if (jm.joint_trajectory_topic.empty()) {continue;}

        joint_mappings[joint_name] = jm;
        joint_pub[jm.joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          jm.joint_trajectory_topic, 10);
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu joint parameters from rosparam", joint_mappings.size());
  }

  // Load pose parameters. "trigger" is an optional modifier button held while
  // pressing the pose button; omit it to bind the pose button on its own.
  if (has_param("control_poses.pose_list")) {
    get_param("control_poses.pose_list", pose_list);
    for (const auto & pose_name : pose_list) {
      PoseMap pm{};
      pm.pose_name = pose_name;
      get_param("control_poses.trigger", pm.trigger);
      get_param("control_poses." + pose_name + ".button", pm.button);

      const std::string base = "control_poses." + pose_name;
      // Shared default first, then the per-pose override.
      get_param("control_poses.time_from_start", pm.time_from_start);
      get_param(base + ".time_from_start", pm.time_from_start);

      // A pose defined in YAML lists the groups it drives; each group names the
      // robot.yaml joint group whose trajectory topic carries it.
      std::vector<std::string> groups;
      get_param(base + ".groups", groups);

      // Single-group shorthand: joints/positions directly under the pose.
      if (groups.empty() && has_param(base + ".joints")) {
        groups.push_back("");
      }

      for (const auto & g : groups) {
        const std::string gbase = g.empty() ? base : base + "." + g;
        mark_visited(gbase);
        PoseJointGroup pg{};
        get_param(gbase + ".joints", pg.joint_names);
        get_param(gbase + ".positions", pg.positions);

        if (pg.joint_names.empty()) {
          RCLCPP_ERROR(get_logger(),
            "Pose '%s' group '%s' lists no joints — skipping that group",
            pose_name.c_str(), g.c_str());
          continue;
        }
        if (pg.joint_names.size() != pg.positions.size()) {
          RCLCPP_ERROR(get_logger(),
            "Pose '%s' group '%s': %zu joints but %zu positions — skipping that group",
            pose_name.c_str(), g.c_str(), pg.joint_names.size(), pg.positions.size());
          continue;
        }

        get_param(gbase + ".button", pg.button);
        // Topic: explicit override, else the joint group's robot.yaml topic.
        get_param(gbase + ".joint_trajectory_topic", pg.joint_trajectory_topic);
        if (pg.joint_trajectory_topic.empty() && !g.empty()) {
          get_param("robot_topic_name.joint_trajectory_topic." + g,
              pg.joint_trajectory_topic);
        }
        if (pg.joint_trajectory_topic.empty()) {
          RCLCPP_ERROR(get_logger(),
            "Pose '%s' group '%s': no trajectory topic (set joint_trajectory_topic or "
            "name a group from robot_topic_name) — skipping that group",
            pose_name.c_str(), g.c_str());
          continue;
        }

        if (joint_pub.find(pg.joint_trajectory_topic) == joint_pub.end()) {
          joint_pub[pg.joint_trajectory_topic] =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            pg.joint_trajectory_topic, 10);
        }
        pm.joint_groups.push_back(pg);
      }

      pose_mappings.push_back(pm);
      RCLCPP_INFO(get_logger(), "Pose '%s': %s", pose_name.c_str(),
        pm.joint_groups.empty() ?
            "no joints defined — using MoveToPose action" :
            "defined in YAML — publishing joint trajectories");
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose parameters from rosparam", pose_mappings.size());
  }

  // Load cmd_vel parameters. Either button-based or axis-based enable is allowed.
  if (has_param("control_velocity.button") ||
    has_param("control_velocity.axis"))
  {
    get_param("control_velocity.button", cvm.button);
    get_param("control_velocity.fast_button", cvm.fast_button);
    get_param("control_velocity.axis", cvm.axis);
    get_param("control_velocity.fast_axis", cvm.fast_axis);
    get_param("control_velocity.linear_x_axis", cvm.linear_x_axis);
    get_param("control_velocity.linear_y_axis", cvm.linear_y_axis);
    get_param("control_velocity.angular_axis", cvm.angular_axis);
    get_param("control_velocity.axis_sign", cvm.axis_sign);
    get_param("control_velocity.linear_scale", cvm.linear_scale);
    get_param("control_velocity.angular_scale", cvm.angular_scale);
    get_param("control_velocity.fast_linear_scale", cvm.fast_linear_scale);
    get_param("control_velocity.fast_angular_scale", cvm.fast_angular_scale);
    RCLCPP_INFO(get_logger(), "Loaded control_velocity parameters from rosparam");
  }

  // Blends: drive a joint group between two configurations from any input.
  std::vector<std::string> blend_names;
  if (get_param("control_blends.blends_name", blend_names)) {
    for (const auto & bname : blend_names) {
      const std::string base = "control_blends." + bname;
      BlendMap bm;
      bm.name = bname;
      get_param(base + ".enable_axis", bm.enable_axis);
      get_param(base + ".enable_button", bm.enable_button);
      get_param(base + ".axis", bm.axis);
      get_param(base + ".axis_sign", bm.axis_sign);
      get_param(base + ".close_button", bm.close_button);
      get_param(base + ".open_button", bm.open_button);
      get_param(base + ".speed", bm.speed);

      std::string group;
      get_param(base + ".group", group);
      get_param(base + ".joint_trajectory_topic", bm.joint_trajectory_topic);
      if (bm.joint_trajectory_topic.empty() && !group.empty()) {
        bm.joint_trajectory_topic = group_trajectory_topic(group);
      }

      // Endpoints: `open`/`close` name a control_poses entry; otherwise fall
      // back to per-joint open_pos/close_pos under joints_name.
      BlendEndpoint open_ep, close_ep;
      const bool have_open = resolve_blend_endpoint(base, "open", group, open_ep);
      const bool have_close = resolve_blend_endpoint(base, "close", group, close_ep);

      if (have_open && have_close) {
        if (open_ep.joint_names != close_ep.joint_names ||
          open_ep.positions.size() != open_ep.joint_names.size() ||
          close_ep.positions.size() != close_ep.joint_names.size())
        {
          RCLCPP_ERROR(get_logger(),
            "Blend '%s': open and close must cover the same joints — skipping",
            bname.c_str());
          continue;
        }
        for (size_t i = 0; i < open_ep.joint_names.size(); ++i) {
          bm.joints.push_back({open_ep.joint_names[i], open_ep.positions[i],
              close_ep.positions[i]});
        }
      } else {
        std::vector<std::string> jnames;
        get_param(base + ".joints_name", jnames);
        for (const auto & jn : jnames) {
          BlendJoint bj;
          bj.name = jn;
          get_param(base + "." + jn + ".open_pos", bj.open_pos);
          get_param(base + "." + jn + ".close_pos", bj.close_pos);
          bm.joints.push_back(bj);
        }
      }

      if (bm.joints.empty() || bm.joint_trajectory_topic.empty()) {
        RCLCPP_ERROR(get_logger(),
          "Blend '%s' needs joints_name and a group or joint_trajectory_topic — skipping",
          bname.c_str());
        continue;
      }
      if (joint_pub.find(bm.joint_trajectory_topic) == joint_pub.end()) {
        joint_pub[bm.joint_trajectory_topic] =
          this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          bm.joint_trajectory_topic, 10);
      }
      blend_mappings.push_back(bm);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu blend(s) from rosparam", blend_mappings.size());
  }
  // Load quest parameters
  if (has_param("quest_control.groups")) {
    get_param("quest_control.groups", quest_groups);
    for (const auto & group : quest_groups) {
      // A tracked group lists its joints in `names`, then describes each one
      // below it — the same shape as control_joints and the hand's adaptive block.
      std::vector<std::string> joint_names;
      get_param("quest_control." + group + ".joints_name", joint_names);

      if (!joint_names.empty()) {
        const auto & overrides =
          this->get_node_parameters_interface()->get_parameter_overrides();
        QuestTrackedGroup g{};
        g.group = group;
        get_param("quest_control." + group + ".enable_axis", g.enable_axis);
        get_param("quest_control." + group + ".target_frame_name", g.target_frame_name);
        get_param("quest_control." + group + ".motion_scale", g.motion_scale);

        for (const auto & jname : joint_names) {
          const std::string jprefix = "quest_control." + group + "." + jname;
          std::string type, axis;
          get_param(jprefix + ".type", type);
          get_param(jprefix + ".axis", axis);
          // sign: YAML "1"/"-1" parses as an integer override; read via the raw
          // override value so both int and double authoring styles work.
          double sign = 1.0;
          const std::string sign_key = jprefix + ".sign";
          read_keys_.insert(sign_key);
          auto sign_it = overrides.find(sign_key);
          if (sign_it != overrides.end()) {
            sign = (sign_it->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) ?
              static_cast<double>(sign_it->second.get<int64_t>()) :
              sign_it->second.get<double>();
          }

          TrackedJoint tj;
          tj.name = jname;
          tj.sign = sign;
          if (type == "rotation") {
            tj.prismatic = false;
            if (axis == "roll") {tj.component = 0;} else if (axis == "pitch") {
              tj.component = 1;
            } else if (axis == "yaw") {tj.component = 2;} else {
              RCLCPP_ERROR(get_logger(),
                "Quest group '%s' joint '%s': invalid rotation axis '%s' — skipping joint",
                group.c_str(), jname.c_str(), axis.c_str());
              continue;
            }
          } else if (type == "prismatic") {
            tj.prismatic = true;
            if (axis == "x") {tj.component = 0;} else if (axis == "y") {
              tj.component = 1;
            } else if (axis == "z") {tj.component = 2;} else {
              RCLCPP_ERROR(get_logger(),
                "Quest group '%s' joint '%s': invalid prismatic axis '%s' — skipping joint",
                group.c_str(), jname.c_str(), axis.c_str());
              continue;
            }
          } else {
            RCLCPP_ERROR(get_logger(),
              "Quest group '%s' joint '%s': unknown type '%s' — skipping joint",
              group.c_str(), jname.c_str(), type.c_str());
            continue;
          }
          g.joints.push_back(tj);
        }

        if (g.joints.empty()) {
          RCLCPP_ERROR(get_logger(), "Quest group '%s' has no usable joints — skipping",
            group.c_str());
          continue;
        }

        g.joint_trajectory_topic = group_trajectory_topic(group);
        if (g.joint_trajectory_topic.empty()) {continue;}

        g.latched_positions.assign(g.joints.size(), 0.0);
        joint_pub[g.joint_trajectory_topic] =
          this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          g.joint_trajectory_topic, 10);
        quest_tracked_groups[group] = g;
        continue;
      }

      // Group type is inferred from which fields are present.
      const bool is_arm = has_param("quest_control." + group +
          ".end_effector_frame_name");
      const bool is_hand = has_param("quest_control." + group + ".pose_action") ||
        has_param("quest_control." + group + ".adaptive.trigger_axis");

      if (is_arm) {
        QuestArmMap am{};
        am.group = group;
        get_param("quest_control." + group + ".end_effector_frame_name",
            am.end_effector_frame_name);
        get_param("quest_control." + group + ".target_frame_name", am.target_frame_name);
        get_param("quest_control." + group + ".motion_scale", am.motion_scale);
        get_param("quest_control." + group + ".enable_axis", am.enable_axis);
        am.arm_joint_trajectory_topic = group_trajectory_topic(group);
        if (am.arm_joint_trajectory_topic.empty()) {continue;}
        // Optional proximity thresholds — defaults are set in the struct
        if (has_param("quest_control." + group + ".proximity_threshold")) {
          get_param("quest_control." + group + ".proximity_threshold",
              am.proximity_threshold);
        }
        if (has_param("quest_control." + group + ".proximity_angle_threshold")) {
          get_param("quest_control." + group + ".proximity_angle_threshold",
              am.proximity_angle_threshold);
        }

        // Frames default from the controller side so older configs still work.
        get_param("quest_control." + group + ".controller_frame_name",
            am.controller_frame_name);
        get_param("quest_control." + group + ".controller_echo_frame_name",
            am.controller_echo_frame_name);
        // Log label only; the frames below carry the real identity.
        am.controller = group;

        if (am.controller_frame_name.empty() || am.controller_echo_frame_name.empty()) {
          RCLCPP_ERROR(get_logger(),
            "Quest arm '%s' needs controller_frame_name and "
            "controller_echo_frame_name — skipping", group.c_str());
          continue;
        }

        joint_pub[am.arm_joint_trajectory_topic] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          am.arm_joint_trajectory_topic, 10);
        quest_arm_mappings[group] = am;
        // One tracking-state entry per distinct controller side.
        auto & st = arm_track_[am.controller_frame_name];
        st.controller_frame_name = am.controller_frame_name;
        st.controller_echo_frame_name = am.controller_echo_frame_name;
      } else if (is_hand) {
        QuestHandMap hm{};
        hm.group = group;
        get_param("quest_control." + group + ".speed", hm.speed);

        if (has_param("quest_control." + group + ".single_joint.axis")) {
          get_param("quest_control." + group + ".single_joint.axis", hm.type_axis);
          get_param("quest_control." + group + ".single_joint.name", hm.type_joint);
          if (has_param("quest_control." + group + ".single_joint.axis_sign")) {
            get_param("quest_control." + group + ".single_joint.axis_sign", hm.type_sign);
          }
          if (has_param("quest_control." + group + ".single_joint.min")) {
            get_param("quest_control." + group + ".single_joint.min", hm.type_min);
          }
          if (has_param("quest_control." + group + ".single_joint.max")) {
            get_param("quest_control." + group + ".single_joint.max", hm.type_max);
          }
        }
        if (has_param("quest_control." + group + ".pose_button")) {
          get_param("quest_control." + group + ".pose_button", hm.pose_button);
        }
        if (has_param("quest_control." + group + ".pose_open")) {
          get_param("quest_control." + group + ".pose_open", hm.pose_open);
          get_param("quest_control." + group + ".pose_close", hm.pose_close);
          get_param("quest_control." + group + ".pose_action", hm.pose_action);
        }
        if (has_param("quest_control." + group + ".adaptive.trigger_axis")) {
          get_param("quest_control." + group + ".adaptive.trigger_axis",
              hm.adaptive_trigger_axis);
          get_param("quest_control." + group + ".adaptive.stick_axis",
              hm.adaptive_stick_axis);
          get_param("quest_control." + group + ".adaptive.axis_sign",
              hm.adaptive_close_sign);

          // Load adaptive joint list: adaptive.names is a list of joint names,
          // each with close_pos, open_pos, and optional fixed flag.
          const std::string aj_prefix = "quest_control." + group + ".adaptive";
          if (has_param(aj_prefix + ".joints_name")) {
            std::vector<std::string> aj_names;
            get_param(aj_prefix + ".joints_name", aj_names);
            for (const auto & jname : aj_names) {
              mark_visited(aj_prefix + "." + jname);
              AdaptiveJointTarget ajt;
              ajt.name = jname;
              get_param(aj_prefix + "." + jname + ".close_pos", ajt.close_pos);
              get_param(aj_prefix + "." + jname + ".open_pos", ajt.open_pos);
              if (has_param(aj_prefix + "." + jname + ".fixed")) {
                get_param(aj_prefix + "." + jname + ".fixed", ajt.fixed);
              }
              hm.adaptive_joints.push_back(ajt);
            }
          }
        }
        quest_hand_mappings[group] = hm;
      } else {
        mark_visited("quest_control." + group);
        RCLCPP_WARN(get_logger(), "Quest group '%s' is neither arm nor hand — skipping",
            group.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu quest arm and %zu quest hand parameters from rosparam",
      quest_arm_mappings.size(), quest_hand_mappings.size());
    has_quest_controls = !quest_groups.empty();

    // Create one enable-publisher per arm (planning group)
    for (const auto & [arm_name, am] : quest_arm_mappings) {
      if (arm_track_pubs_.find(am.group) == arm_track_pubs_.end()) {
        // transient_local so a late-starting subscriber (e.g. the Servo bridge)
        // still receives the current enable state.
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
        hand_pose_clients_.find(hm.group) == hand_pose_clients_.end())
      {
        hand_pose_clients_[hm.group] =
          rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>(
            this, hm.pose_action);
        hand_open_state_[hm.group] = true;
        hand_toggle_time_[hm.group] = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(get_logger(),
          "Created hand pose client for '%s' → '%s'",
          hm.group.c_str(), hm.pose_action.c_str());
      }
    }
  }

  requires_joint_states = !joint_mappings.empty() || has_quest_controls;

  report_config_summary();

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

  warn_unknown_parameters();
}

// Warn about config keys nothing read. A key is accepted if the code entered
// its subtree (visited_prefixes_) — those are groups left deliberately empty.
void SOBITSTeleop::warn_unknown_parameters()
{
  static const char * kPrefixes[] = {
    "control_joints.", "control_poses.", "control_velocity.",
    "quest_control.", "robot_topic_name."
  };

  const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    (void)value;
    bool matches_prefix = false;
    for (const char * prefix : kPrefixes) {
      if (name.rfind(prefix, 0) == 0) {matches_prefix = true; break;}
    }
    if (!matches_prefix) {continue;}
    if (read_keys_.count(name)) {continue;}

    bool under_visited = false;
    for (const auto & prefix : visited_prefixes_) {
      if (name.rfind(prefix + ".", 0) == 0) {under_visited = true; break;}
    }
    if (under_visited) {continue;}

    RCLCPP_WARN(get_logger(),
      "Unknown parameter '%s' - check for a typo; it has no effect", name.c_str());
  }
}

void SOBITSTeleop::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // position can be shorter than name (velocity/effort-only publishers).
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; i++) {
    joint_pos[msg->name[i]] = msg->position[i];
  }
  if (n > 0) {joint_state_initialized = true;}
}

void SOBITSTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // previous_buttons is updated at the end of teleop() only, so a press edge
  // survives until the next tick even if several joy messages arrive between ticks.
  latest_axes = msg->axes;
  latest_buttons = msg->buttons;
  joy_received = true;
}


void SOBITSTeleop::robot_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // Re-stamp every transform with arrival wall time: sources use skewed clocks (sim time,
  // Quest up to ~4 min ahead), so "newest" must mean "most recently received".
  const rclcpp::Time now_wall = wall_clock_->now();

  // Offset each transform by i ns so repeated frames in one message don't collide.
  uint32_t i = 0;
  for (const auto & t : msg->transforms) {
    geometry_msgs::msg::TransformStamped ts = t;
    ts.header.stamp = now_wall + rclcpp::Duration(0, i++);
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


bool SOBITSTeleop::send_pose(const PoseMap & pose_map, const PoseJointGroup * only)
{
  // YAML-defined pose: publish straight to the controllers, no action server.
  if (!pose_map.joint_groups.empty()) {
    for (const auto & pg : pose_map.joint_groups) {
      if (only && &pg != only) {continue;}
      auto it = joint_pub.find(pg.joint_trajectory_topic);
      if (it == joint_pub.end()) {continue;}

      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      traj.joint_names = pg.joint_names;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = pg.positions;
      pt.velocities.assign(pg.positions.size(), 0.0);
      pt.time_from_start = rclcpp::Duration::from_seconds(pose_map.time_from_start);
      traj.points.push_back(pt);

      it->second->publish(traj);
    }
    RCLCPP_INFO(get_logger(), "Sending pose '%s' over %zu joint group(s), %.1f s",
      pose_map.pose_name.c_str(), only ? 1u : pose_map.joint_groups.size(),
      pose_map.time_from_start);
    return true;
  }

  if (!move_to_pose_client->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "move_to_pose action server not ready — skipping pose '%s'", pose_map.pose_name.c_str());
    return false;
  }

  auto goal_msg = sobits_interfaces::action::MoveToPose::Goal();
  goal_msg.pose_name = pose_map.pose_name;
  goal_msg.time_allowance.sec = 10;

  auto send_goal_options =
    rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this,
      pose_name = pose_map.pose_name](const auto & result) {
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
  return true;
}

// out      = T(base_footprint <- quest_frame)  — used for arm target computation
// out_base = T(base_footprint <- quest_frame)  — used for RViz re-broadcast under base_footprint
bool SOBITSTeleop::lookup_quest_frame(
  const std::string & quest_frame,
  tf2::Transform & out,
  tf2::Transform * out_base)
{
  try {
    auto ts = tf_buffer->lookupTransform(base_frame, quest_frame, tf2::TimePointZero,
      tf2::Duration(0));
    // Reject stamps far from the wall clock: TimePointZero serves cached transforms
    // forever, so a disconnected headset's last pose would look like live input.
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
    if (out_base) {*out_base = T_base_quest;}
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
      "%s TF lookup failed: %s", quest_frame.c_str(), ex.what());
    return false;
  }
}

void SOBITSTeleop::process_joints()
{
  std::map<std::string, trajectory_msgs::msg::JointTrajectory> trajs;

  for (auto &[name, m] : joint_mappings) {

    // Either enable source arms the joint; with neither set it is always live.
    const bool gated = m.button >= 0 || m.enable_axis >= 0;
    if (gated && !button_down(m.button) && !axis_held(m.enable_axis)) {continue;}

    float axis_val = axis_value(m.axis);
    if (std::abs(axis_val) < 1e-3) {continue;}

    const bool fast = button_down(m.fast_button) || axis_held(m.fast_axis);

    // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
    double delta_pos = axis_val * m.axis_sign * (fast ? m.fast_speed : m.speed) * jog_tick_scale_;
    double target = joint_pos[m.joint_name] + delta_pos;
    if (!clamp_to_limits_checked(m.joint_name, target)) {continue;}
    joint_pos[m.joint_name] = target;

    auto & traj = trajs[m.joint_trajectory_topic];
    traj.joint_names.push_back(m.joint_name);
    if (traj.points.empty()) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = {joint_pos[m.joint_name]};
      p.time_from_start = rclcpp::Duration::from_seconds(dt());
      traj.points.push_back(p);
    } else {traj.points[0].positions.push_back(joint_pos[m.joint_name]);}
  }

  for (auto & tj : trajs) {
    const auto & joint_trajectory_topic = tj.first;
    auto & traj = tj.second;
    auto it = joint_pub.find(joint_trajectory_topic);
    if (it != joint_pub.end() && traj.joint_names.size() > 0) {it->second->publish(traj);}
  }
}

void SOBITSTeleop::process_poses()
{
  // Skip pose buttons while any arm is latched — a pose trajectory would fight tracking.
  bool any_arm_latched = false;
  for (const auto & kv : arm_track_) {
    any_arm_latched = any_arm_latched || kv.second.latched;
  }

  for (const auto & pose_map : pose_mappings) {
    // A trigger of -1 means no modifier is required; otherwise it must be held.
    if (pose_map.trigger >= 0 && !button_down(pose_map.trigger)) {continue;}

    if (any_arm_latched) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "pose buttons disabled while an arm is latched");
      continue;
    }

    if (button_pressed(pose_map.button)) {
      send_pose(pose_map);
      continue;
    }

    // A group may carry its own button that sends only its part of the pose.
    for (const auto & pg : pose_map.joint_groups) {
      if (button_pressed(pg.button)) {send_pose(pose_map, &pg);}
    }
  }
}

// Steps a blend's joints toward whichever endpoint the input selects, at a
// speed set by how far it is pushed. Holding partway stops there.
void SOBITSTeleop::process_blends()
{
  for (const auto & bm : blend_mappings) {
    const bool gated = bm.enable_axis >= 0 || bm.enable_button >= 0;
    if (gated && !axis_held(bm.enable_axis) && !button_down(bm.enable_button)) {continue;}

    // Deflection: sign selects the endpoint, magnitude scales the step.
    double deflection = axis_value(bm.axis) * bm.axis_sign;
    if (button_down(bm.close_button)) {deflection = 1.0;}
    if (button_down(bm.open_button)) {deflection = -1.0;}
    if (std::abs(deflection) < 0.1) {continue;}

    const bool closing = deflection > 0.0;
    const double step = bm.speed * std::abs(deflection) * jog_tick_scale_;

    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (const auto & bj : bm.joints) {
      const double goal = closing ? bj.close_pos : bj.open_pos;
      double cur = joint_pos.count(bj.name) ? joint_pos.at(bj.name) : goal;
      double next = cur + ((goal > cur) ? step : -step);
      if ((goal > cur && next > goal) || (goal < cur && next < goal)) {next = goal;}
      if (!clamp_to_limits_checked(bj.name, next)) {continue;}
      joint_pos[bj.name] = next;
      traj.joint_names.push_back(bj.name);
      pt.positions.push_back(next);
    }
    if (traj.joint_names.empty()) {continue;}

    pt.time_from_start = rclcpp::Duration::from_seconds(dt());
    traj.points.push_back(pt);
    auto it = joint_pub.find(bm.joint_trajectory_topic);
    if (it != joint_pub.end() && it->second) {it->second->publish(traj);}
  }
}

void SOBITSTeleop::process_cmd_vel()
{
  // Skip entirely if cmd_vel isn't configured — avoids flooding zero twists.
  if (cvm.button >= 0 || cvm.axis >= 0) {
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Twist stop;

    bool cmd_vel_enabled = false;
    bool fast_mode = false;

    if (cvm.button >= 0 &&
      cvm.button < static_cast<int>(latest_buttons.size()))
    {
      cmd_vel_enabled = button_down(cvm.button);
      if (cvm.fast_button >= 0 &&
        cvm.fast_button < static_cast<int>(latest_buttons.size()))
      {
        fast_mode = fast_mode || button_down(cvm.fast_button);
      }
    }
    if (cvm.axis >= 0 &&
      cvm.axis < static_cast<int>(latest_axes.size()))
    {
      // OR with the button branch — either enable source is allowed.
      cmd_vel_enabled = cmd_vel_enabled || axis_held(cvm.axis);
      if (cvm.fast_axis >= 0 &&
        cvm.fast_axis < static_cast<int>(latest_axes.size()))
      {
        fast_mode = fast_mode || axis_held(cvm.fast_axis);
      }
    }

    if (cmd_vel_enabled) {
      const double linear_scale = fast_mode ? cvm.fast_linear_scale : cvm.linear_scale;
      const double angular_scale = fast_mode ? cvm.fast_angular_scale : cvm.angular_scale;

      if (cvm.linear_x_axis >= 0 &&
        cvm.linear_x_axis < static_cast<int>(latest_axes.size()))
      {
        twist.linear.x = axis_value(cvm.linear_x_axis) * linear_scale;
      }
      if (cvm.linear_y_axis >= 0 &&
        cvm.linear_y_axis < static_cast<int>(latest_axes.size()))
      {
        twist.linear.y = axis_value(cvm.linear_y_axis) * linear_scale * cvm.axis_sign;
      }
      if (cvm.angular_axis >= 0 &&
        cvm.angular_axis < static_cast<int>(latest_axes.size()))
      {
        twist.angular.z = axis_value(cvm.angular_axis) * angular_scale * cvm.axis_sign;
      }

      cmd_vel_pub->publish(twist);
    } else if (cmd_vel_was_enabled_) {
      // Publish stop once on the enabled->disabled edge, not every tick.
      cmd_vel_pub->publish(stop);
    }
    cmd_vel_was_enabled_ = cmd_vel_enabled;
  }
}

// Latch/track one quest_control group. Trigger comes from /joy so releasing
// always unlatches, even while the group's own TF is stale.
void SOBITSTeleop::process_tracked_group(QuestTrackedGroup & g)
{
  g.control_enabled = axis_held(g.enable_axis);

  tf2::Transform current_tf;
  const bool tf_ok = lookup_quest_frame(g.target_frame_name, current_tf);

  if (tf_ok) {
    // Re-anchor across a TF gap, else the whole gap delta lands as one jump.
    if (g.tracking && !g.tf_ok_prev) {
      for (size_t i = 0; i < g.joints.size(); ++i) {
        g.latched_positions[i] = joint_pos[g.joints[i].name];
      }
      g.last_tf = current_tf;
    }

    if (g.control_enabled && !g.tracking) {
      for (size_t i = 0; i < g.joints.size(); ++i) {
        g.latched_positions[i] = joint_pos[g.joints[i].name];
      }
      g.last_tf = current_tf;
      g.tracking = true;
      RCLCPP_INFO(this->get_logger(), "%s tracking started", g.group.c_str());
    }

    if (g.tracking) {
      tf2::Transform T_delta = g.last_tf.inverse() * current_tf;
      double rpy[3];
      tf2::Matrix3x3(T_delta.getRotation()).getRPY(rpy[0], rpy[1], rpy[2]);
      const tf2::Vector3 & o = T_delta.getOrigin();
      const double pos[3] = {o.x(), o.y(), o.z()};

      trajectory_msgs::msg::JointTrajectory traj;
      trajectory_msgs::msg::JointTrajectoryPoint p;
      bool all_ok = true;
      for (size_t i = 0; i < g.joints.size(); ++i) {
        const auto & j = g.joints[i];
        const double component = j.prismatic ? pos[j.component] : rpy[j.component];
        double target = g.latched_positions[i] + g.motion_scale * component * j.sign;
        if (!clamp_to_limits_checked(j.name, target)) {all_ok = false; break;}
        traj.joint_names.push_back(j.name);
        p.positions.push_back(target);
      }

      if (all_ok) {
        p.time_from_start = rclcpp::Duration::from_seconds(dt());
        traj.points.push_back(p);

        auto it = joint_pub.find(g.joint_trajectory_topic);
        if (it != joint_pub.end() && it->second) {
          it->second->publish(traj);
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Publisher for %s not found", g.joint_trajectory_topic.c_str());
        }
      }
    }
  }
  g.tf_ok_prev = tf_ok;

  // Outside the TF gate: a stale frame must not strand the latch.
  if (!g.control_enabled && g.tracking) {
    g.tracking = false;
    RCLCPP_INFO(this->get_logger(), "%s tracking stopped", g.group.c_str());
  }
}

void SOBITSTeleop::teleop()
{
  if (!joy_received) {return;}
  if (requires_joint_states && !joint_state_initialized) {return;}

  process_joints();
  process_poses();
  process_cmd_vel();
  process_blends();

  // Quest controllers: Unity publishes Quest frames directly under base_footprint.
  bool base_odom_ok = true;  // always ready; kept as guard variable for structure

  if (this->has_parameter("quest_control.groups")) {
    // Head / HMD — also used as body reference for arm target scaling
    bool head_tf_ok = false;
    tf2::Transform current_tf;  // controller pose this tick, base_footprint
    if (base_odom_ok) {
      tf2::Transform T_base_hmd;
      head_tf_ok = lookup_quest_frame("hmd_odom", current_tf, &T_base_hmd);
      if (head_tf_ok) {
        current_tf_hmd = current_tf;
        current_tf_hmd_odom = T_base_hmd;
        // Re-broadcast under base_footprint (RViz visualization).
        geometry_msgs::msg::TransformStamped hmd_msg;
        hmd_msg.header.stamp = this->now();
        hmd_msg.header.frame_id = base_frame;
        hmd_msg.child_frame_id = "hmd_link";
        hmd_msg.transform = tf2::toMsg(T_base_hmd);
        tf_broadcaster->sendTransform(hmd_msg);
      }
    }

    for (auto & [name, g] : quest_tracked_groups) {
      process_tracked_group(g);
    }

    // Arm. Helper: false if any transform component is NaN/Inf (Quest broadcasts NaN when untracked).
    auto transform_valid = [](const geometry_msgs::msg::Transform & t) {
        const auto & q = t.rotation;
        const auto & v = t.translation;
        return std::isfinite(q.x) && std::isfinite(q.y) &&
               std::isfinite(q.z) && std::isfinite(q.w) &&
               std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
               (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.01;
      };

    // One TF lookup per distinct controller frame, shared by the arms using it.
    auto find_arm = [&](const std::string & frame) -> QuestArmMap * {
        for (auto & [name, m] : quest_arm_mappings) {
          if (m.controller_frame_name == frame) {return &m;}
        }
        return nullptr;
      };

    for (auto & [side, st] : arm_track_) {
      st.tf_ok = false;
      if (!base_odom_ok) {continue;}
      tf2::Transform T_ctrl, T_base_ctrl;
      if (!lookup_quest_frame(st.controller_frame_name, T_ctrl, &T_base_ctrl)) {continue;}
      geometry_msgs::msg::Transform t_msg = tf2::toMsg(T_ctrl);
      if (!transform_valid(t_msg)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
          "%s has invalid (NaN/zero) transform — waiting for controller tracking",
          st.controller_frame_name.c_str());
        continue;
      }
      st.current_tf = T_ctrl;
      st.current_tf_odom = T_base_ctrl;
      st.tf_ok = true;
      // Re-broadcast under base_footprint (RViz visualization).
      geometry_msgs::msg::TransformStamped c_msg;
      c_msg.header.stamp = this->now();
      c_msg.header.frame_id = base_frame;
      c_msg.child_frame_id = st.controller_echo_frame_name;
      c_msg.transform = tf2::toMsg(T_base_ctrl);
      tf_broadcaster->sendTransform(c_msg);
    }

    // Stale/lost input unlatches its arm — a frozen latch would teleport on the next
    // valid frame; re-gripping re-latches with a fresh zero-error capture.
    bool any_latched = false;
    for (auto & [side, st] : arm_track_) {
      if (st.latched && !st.tf_ok) {
        st.latched = false;
        st.have_pub_prev = false;
        if (auto * m = find_arm(side)) {publish_arm_tracking(m->group, false);}
        RCLCPP_WARN(this->get_logger(), "%s controller TF stale/lost — %s arm unlatched",
          side.c_str(), side.c_str());
      }
      any_latched = any_latched || st.latched;
    }
    if (arm_tracking && !any_latched) {
      arm_tracking = false;
      RCLCPP_INFO(this->get_logger(), "Arm tracking stopped (controller input lost)");
    }

    // Stop tracking immediately when every arm's grip button is released.
    if (!any_arm_enable_held() && arm_tracking) {
      arm_tracking = false;
      for (auto & [side, st] : arm_track_) {
        st.latched = false;
        st.have_pub_prev = false;
      }
      RCLCPP_INFO(this->get_logger(), "Arm tracking stopped");
      for (auto & [name, m] : quest_arm_mappings) {
        publish_arm_tracking(m.group, false);
      }
    }
    // Per-arm latching is handled inside process_arm, after the fresh
    // end-effector TF has been read and the proximity check can be done.

    for (auto &[name, m] : quest_arm_mappings) {
      auto st_it = arm_track_.find(m.controller_frame_name);
      if (st_it == arm_track_.end()) {continue;}
      auto & st = st_it->second;
      if (st.tf_ok && head_tf_ok) {
        process_arm(m, st, head_tf_ok, axis_held(m.enable_axis));
      }
    }// Arm

    // Gripper — separate loop over hand groups, runs after the arm loop.
    for (auto &[name, m] : quest_hand_mappings) {
      process_hand(name, m);
    }
  }// Quest controllers

  // Consume edges once per tick (teleop() runs faster than joy publishes).
  previous_buttons = latest_buttons;
}

void SOBITSTeleop::process_hand(const std::string & name, QuestHandMap & m)
{
    // ── 1. Hand pose toggle (open / close) on button press ───────────────
    // Configured via pose_button / pose_open / pose_close / pose_action in quest.yaml.
  auto hp_client_it = hand_pose_clients_.find(name);
  if (m.pose_button >= 0 && hp_client_it != hand_pose_clients_.end()) {
    rclcpp::Time & toggle_time = hand_toggle_time_.at(name);
    const bool debounce_ok = (this->now() - toggle_time).seconds() > 0.4;

    if (button_pressed(m.pose_button) && debounce_ok) {
      auto & client = hp_client_it->second;
            // Check server readiness before flipping state, non-blocking.
      if (client->action_server_is_ready()) {
        toggle_time = this->now();
        bool & is_open = hand_open_state_.at(name);
        is_open = !is_open;
        const std::string pose_name = is_open ? m.pose_open : m.pose_close;

        auto goal = sobits_interfaces::action::MoveToPose::Goal();
        goal.pose_name = pose_name;
              // time_allowance becomes the trajectory's time_from_start, i.e. the motion duration.
        goal.time_allowance.sec = 1;
              // Suppress this hand's adaptive goal stream until the pose motion
              // completes, so per-tick "hold" goals don't fight the trajectory.
        m.hand_pose_in_flight = true;
        m.hand_pose_deadline = this->now() + rclcpp::Duration::from_seconds(2.0);
        bool * pose_in_flight = &m.hand_pose_in_flight;
        auto opts =
          rclcpp_action::Client<sobits_interfaces::action::MoveToPose>::SendGoalOptions();
        opts.goal_response_callback =
          [pose_in_flight](rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveToPose>::
          SharedPtr h) {
            if (!h) {*pose_in_flight = false;}      // rejected: resume adaptive
          };
        opts.result_callback = [this, pose_name, pose_in_flight](const auto & result) {
            *pose_in_flight = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_INFO(get_logger(), "Hand pose '%s' succeeded", pose_name.c_str());
            } else {
              RCLCPP_WARN(get_logger(), "Hand pose '%s' failed", pose_name.c_str());
            }
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
        // A pose toggle still executing takes precedence over the adaptive stream.
  if (m.hand_pose_in_flight && this->now() >= m.hand_pose_deadline) {
    m.hand_pose_in_flight = false;
  }
  if (m.adaptive_trigger_axis >= 0 && !m.adaptive_joints.empty() &&
    !m.hand_pose_in_flight &&
    m.adaptive_trigger_axis < static_cast<int>(latest_axes.size()) &&
    m.adaptive_stick_axis < static_cast<int>(latest_axes.size()) &&
    latest_axes[m.adaptive_trigger_axis] > 0.1)
  {
    float raw_stick = latest_axes[m.adaptive_stick_axis];
    float close_frac = std::clamp(raw_stick * m.adaptive_close_sign, 0.0f, 1.0f);
    float open_frac = std::clamp(-raw_stick * m.adaptive_close_sign, 0.0f, 1.0f);
    float deflection = std::max(close_frac, open_frac);

          // Vertical stick rotates the grip-type knuckle while the trigger is held.
          // Small deadzone rejects noise; rescaled past it so the step ramps from zero.
    constexpr float kVjogDeadzone = 0.15f;
    float vjog_stick = 0.0f;
    if (m.type_axis >= 0 && m.type_axis < static_cast<int>(latest_axes.size())) {
      const float raw = latest_axes[m.type_axis];
      if (std::abs(raw) > kVjogDeadzone) {
        vjog_stick = std::copysign(
          (std::abs(raw) - kVjogDeadzone) / (1.0f - kVjogDeadzone), raw);
      }
    }

          // Endpoint-swing mode (single_joint.min/max set): the servo can't break the gear
          // spring with small errors, so one flick = one full swing; recenter stick to re-arm.
    const bool vjog_has_range = (m.type_min > -1e8f && m.type_max < 1e8f);
          // Fire only on a decisive, dominantly-vertical push — the same stick's horizontal
          // axis drives open/close and must not flip the 2f/3f pose mid-grasp.
    constexpr float kFlickThreshold = 0.6f;
    float vjog_h = 0.0f;
    if (m.adaptive_stick_axis >= 0 &&
      m.adaptive_stick_axis < static_cast<int>(latest_axes.size()))
    {
      vjog_h = latest_axes[m.adaptive_stick_axis];
    }
    const float vjog_raw =
      (m.type_axis >= 0 && m.type_axis < static_cast<int>(latest_axes.size())) ?
      latest_axes[m.type_axis] : 0.0f;
    const bool vjog_decisive = std::abs(vjog_raw) > kFlickThreshold &&
      std::abs(vjog_raw) > std::abs(vjog_h);
    const bool vjog_fire = vjog_has_range && vjog_decisive && m.vjog_armed;
    if (vjog_has_range && vjog_stick == 0.0f) {m.vjog_armed = true;}
          // Keep goals flowing while a swing is in progress even with the stick
          // released — the endpoint command must persist until arrival.
    const bool vjog_request =
      vjog_has_range ? (vjog_fire || m.vjog_swing_active) : (vjog_stick != 0.0f);

    if (deflection >= 0.1f || vjog_request) {
      const bool closing = (close_frac >= open_frac);
            // Config speeds are radians per legacy 50 ms tick — scale to the actual loop rate.
      float step = m.speed * deflection * static_cast<float>(jog_tick_scale_);
      float vjog_step = m.speed * vjog_stick * static_cast<float>(m.type_sign) *
        static_cast<float>(jog_tick_scale_);

            // A flick starts a persistent swing: every goal re-commands the endpoint until
            // arrival, so open/close rides in the same goals instead of blocking behind one.
      if (vjog_fire) {
        m.vjog_armed = false;
        m.vjog_swing_active = true;
        m.vjog_swing_target = (vjog_step > 0.0f) ? m.type_max : m.type_min;
        m.vjog_swing_deadline = this->now() + rclcpp::Duration::from_seconds(2.5);
      }
      if (m.vjog_swing_active) {
              // Only check arrival if the joint is known; else rely on the deadline.
        const bool arrived = joint_pos.count(m.type_joint) &&
          std::abs(static_cast<float>(joint_pos.at(m.type_joint)) - m.vjog_swing_target) <
          0.08f;
        if (arrived || this->now() >= m.vjog_swing_deadline) {
          m.vjog_swing_active = false;
        }
      }

      auto clamp_to_limits = [&](const std::string & jname, float value) -> float {
          if (!joint_limits.count(jname)) {return value;}
          return std::clamp(value,
                static_cast<float>(std::min(joint_limits.at(jname).lower,
                joint_limits.at(jname).upper)),
                static_cast<float>(std::max(joint_limits.at(jname).lower,
                joint_limits.at(jname).upper)));
        };

      auto step_toward = [&](const std::string & jname, float target) -> float {
          if (joint_pos.find(jname) == joint_pos.end()) {return target;}
          float cur = static_cast<float>(joint_pos.at(jname));
          float dir = (target > cur) ? 1.0f : -1.0f;
          float next = cur + dir * step;
          if ((dir > 0 && next > target) || (dir < 0 && next < target)) {next = target;}
          return clamp_to_limits(jname, next);
        };

      auto goal = sobits_interfaces::action::MoveJoint::Goal();
      float max_delta = 0.f;
      for (const auto & ajt : m.adaptive_joints) {
        goal.target_joint_names.push_back(ajt.name);
        float cur = joint_pos.count(ajt.name) ?
          static_cast<float>(joint_pos.at(ajt.name)) : ajt.close_pos;
        float tgt;
        if (ajt.fixed) {
                // Fixed joints hold position — except the grip-type knuckle, driven by the stick.
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
          goal.target_joint_rad.push_back(cur);        // vertical-only: hold curl
        }
        max_delta = std::max(max_delta, std::abs(tgt - cur));
      }
            // Scale duration to the largest excursion so a swing isn't ballistic.
      constexpr double kMaxHandJointVel = 3.0;        // rad/s — hand servo jog ceiling
      const double goal_sec = std::max(1.0 / teleop_rate_hz, max_delta / kMaxHandJointVel);
      goal.time_allowance = rclcpp::Duration::from_seconds(goal_sec);

            // One jog goal in flight per hand (server runs goals unpreempted on detached
            // threads; a global gate made hands block each other). Deadline backstops a lost result.
      if (move_joint_client->action_server_is_ready() &&
        (!m.jog_goal_in_flight || this->now() >= m.jog_goal_deadline))
      {
        m.jog_goal_in_flight = true;
              // Backstop must outlive the goal's own allowance.
        m.jog_goal_deadline = this->now() +
          rclcpp::Duration::from_seconds(std::max(1.0, goal_sec + 0.5));
        bool * in_flight = &m.jog_goal_in_flight;
        auto opts =
          rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SendGoalOptions();
        opts.goal_response_callback =
          [in_flight](rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveJoint>::
          SharedPtr h) {
            if (!h) {*in_flight = false;}      // rejected: free the slot
          };
        opts.result_callback = [in_flight](const auto &) {*in_flight = false;};
        move_joint_client->async_send_goal(goal, opts);
      }
    }
  }
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::SOBITSTeleop)
