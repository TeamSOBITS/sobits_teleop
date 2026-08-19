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

// Reports which optional config blocks this device.yaml supplies, so a block
// missing on one robot is visible at startup instead of silently doing nothing.
const PoseMap * SOBITSTeleop::find_pose(const std::string & pose)
{
  for (const auto & pm : pose_mappings) {
    if (pm.pose_name == pose) {return &pm;}
  }
  return nullptr;
}

std::string SOBITSTeleop::topic_group_name(const std::string & topic)
{
  const std::string prefix = "robot_topic_name.joint_trajectory_topic.";
  const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    if (name.rfind(prefix, 0) != 0) {continue;}
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
      value.get<std::string>() == topic)
    {
      return name.substr(prefix.size());
    }
  }
  return std::string();
}

void SOBITSTeleop::report_config_summary()
{
  const std::pair<const char *, bool> blocks[] = {
    {"control_joints", !joint_mappings.empty()},
    {"control_poses", !pose_mappings.empty()},
    {"control_velocity", cvm.button >= 0 || cvm.axis >= 0},
    {"control_tracking", !quest_tracked_groups.empty()},
    {"control_cartesian", has_cartesian_groups},
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
  if (has_param("control_joints.groups_name")) {
    get_param("control_joints.groups_name", joint_groups);

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
        get_param("control_joints." + joint_group + "." + joint_name + ".dominant_over",
            jm.dominant_over);
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
  if (has_param("control_poses.poses.poses_name")) {
    get_param("control_poses.poses.poses_name", poses_name);
    for (const auto & pose_name : poses_name) {
      PoseMap pm{};
      pm.pose_name = pose_name;
      get_param("control_poses.trigger", pm.trigger);
      get_param("control_poses.poses." + pose_name + ".button", pm.button);

      const std::string base = "control_poses.poses." + pose_name;
      // Shared default first, then the per-pose override.
      get_param("control_poses.time_from_start", pm.time_from_start);
      get_param(base + ".time_from_start", pm.time_from_start);

      // A pose defined in YAML lists the groups it drives; each group names the
      // robot.yaml joint group whose trajectory topic carries it.
      std::vector<std::string> groups;
      get_param(base + ".groups_name", groups);

      // Single-group shorthand: joints/positions directly under the pose.
      if (groups.empty() && has_param(base + ".joints_name")) {
        groups.push_back("");
      }

      for (const auto & g : groups) {
        const std::string gbase = g.empty() ? base : base + "." + g;
        mark_visited(gbase);
        PoseJointGroup pg{};
        get_param(gbase + ".joints_name", pg.joint_names);
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

  // Blends sweep one group between two poses, so they load after the poses.
  std::vector<std::string> blend_names;
  if (get_param("control_poses.blends.blends_name", blend_names)) {
    for (const auto & bname : blend_names) {
      const std::string base = "control_poses.blends." + bname;
      PoseBlendMap bl;
      bl.name = bname;
      get_param(base + ".enable_axis", bl.enable_axis);
      get_param(base + ".enable_button", bl.enable_button);
      get_param(base + ".axis", bl.axis);
      get_param(base + ".axis_sign", bl.axis_sign);
      get_param(base + ".to_button", bl.to_button);
      get_param(base + ".from_button", bl.from_button);
      get_param(base + ".speed", bl.speed);

      std::vector<std::string> exclude;
      std::string from_pose, to_pose;
      get_param(base + ".exclude_groups", exclude);
      get_param(base + ".from", from_pose);
      get_param(base + ".to", to_pose);

      const PoseMap * from_pm = find_pose(from_pose);
      const PoseMap * to_pm = find_pose(to_pose);
      if (!from_pm || !to_pm) {
        RCLCPP_ERROR(get_logger(), "Blend '%s': pose '%s' or '%s' is not defined — skipping",
          bname.c_str(), from_pose.c_str(), to_pose.c_str());
        continue;
      }

      // Every group both poses define, minus the excluded ones.
      for (const auto & fg : from_pm->joint_groups) {
        if (std::find(exclude.begin(), exclude.end(),
          topic_group_name(fg.joint_trajectory_topic)) != exclude.end())
        {
          continue;
        }
        const PoseJointGroup * tg = nullptr;
        for (const auto & g : to_pm->joint_groups) {
          if (g.joint_trajectory_topic == fg.joint_trajectory_topic) {tg = &g; break;}
        }
        if (!tg) {continue;}

        PoseBlendGroup bg;
        bg.joint_trajectory_topic = fg.joint_trajectory_topic;
        for (size_t i = 0; i < fg.joint_names.size(); ++i) {
          for (size_t k = 0; k < tg->joint_names.size(); ++k) {
            if (tg->joint_names[k] != fg.joint_names[i]) {continue;}
            if (i >= fg.positions.size() || k >= tg->positions.size()) {break;}
            bg.joint_names.push_back(fg.joint_names[i]);
            bg.from_positions.push_back(fg.positions[i]);
            bg.to_positions.push_back(tg->positions[k]);
            break;
          }
        }
        if (!bg.joint_names.empty()) {bl.groups.push_back(bg);}
      }
      if (bl.groups.empty()) {
        RCLCPP_ERROR(get_logger(), "Blend '%s': poses share no joints — skipping",
          bname.c_str());
        continue;
      }
      pose_blends.push_back(bl);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose blend(s) from rosparam", pose_blends.size());
  }

  std::vector<std::string> cycle_names;
  if (get_param("control_poses.cycles.cycles_name", cycle_names)) {
    for (const auto & cname : cycle_names) {
      const std::string base = "control_poses.cycles." + cname;
      PoseCycleMap pc;
      pc.name = cname;
      get_param(base + ".button", pc.button);
      get_param(base + ".exclude_groups", pc.exclude_groups);
      get_param(base + ".poses", pc.poses);

      bool all_defined = true;
      for (const auto & pn : pc.poses) {
        if (!find_pose(pn)) {all_defined = false; break;}
      }
      if (pc.poses.size() < 2 || !all_defined) {
        RCLCPP_ERROR(get_logger(),
          "Cycle '%s': needs two or more poses that are all defined — skipping",
          cname.c_str());
        continue;
      }
      pose_cycles.push_back(pc);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu pose cycle(s) from rosparam", pose_cycles.size());
  }

  // Load cmd_vel parameters. Either button-based or axis-based enable is allowed.
  if (has_param("control_velocity.enable_button") ||
    has_param("control_velocity.enable_axis"))
  {
    get_param("control_velocity.enable_button", cvm.button);
    get_param("control_velocity.fast_enable_button", cvm.fast_button);
    get_param("control_velocity.enable_axis", cvm.axis);
    get_param("control_velocity.fast_enable_axis", cvm.fast_axis);
    get_param("control_velocity.axis_sign", cvm.axis_sign);
    // Robot limits live in robot.yaml; the scales below are fractions of them.
    get_param("base_max_speed.linear", cvm.linear_max);
    get_param("base_max_speed.angular", cvm.angular_max);
    get_param("control_velocity.linear.x_axis", cvm.linear_x_axis);
    get_param("control_velocity.linear.y_axis", cvm.linear_y_axis);
    get_param("control_velocity.linear.scale", cvm.linear_scale);
    get_param("control_velocity.linear.fast_scale", cvm.fast_linear_scale);
    get_param("control_velocity.angular.axis", cvm.angular_axis);
    get_param("control_velocity.angular.scale", cvm.angular_scale);
    get_param("control_velocity.angular.fast_scale", cvm.fast_angular_scale);
    RCLCPP_INFO(get_logger(), "Loaded control_velocity parameters from rosparam");
  }
  // Tracked groups: joints follow a frame's motion since the latch.
  std::vector<std::string> tracking_groups;
  if (has_param("control_tracking.groups_name")) {
    get_param("control_tracking.groups_name", tracking_groups);
    for (const auto & group : tracking_groups) {
      // A group lists its joints in joints_name, then describes each one below.
      std::vector<std::string> joint_names;
      get_param("control_tracking." + group + ".joints_name", joint_names);

      if (joint_names.empty()) {
        RCLCPP_ERROR(get_logger(),
          "Tracking group '%s' lists no joints_name — skipping", group.c_str());
        continue;
      }
      {
        const auto & overrides =
          this->get_node_parameters_interface()->get_parameter_overrides();
        QuestTrackedGroup g{};
        g.group = group;
        get_param("control_tracking." + group + ".enable_axis", g.enable_axis);
        get_param("control_tracking." + group + ".target_frame_name", g.target_frame_name);
        get_param("control_tracking." + group + ".motion_scale", g.motion_scale);

        for (const auto & jname : joint_names) {
          const std::string jprefix = "control_tracking." + group + "." + jname;
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
                "Target group '%s' joint '%s': invalid rotation axis '%s' — skipping joint",
                group.c_str(), jname.c_str(), axis.c_str());
              continue;
            }
          } else if (type == "prismatic") {
            tj.prismatic = true;
            if (axis == "x") {tj.component = 0;} else if (axis == "y") {
              tj.component = 1;
            } else if (axis == "z") {tj.component = 2;} else {
              RCLCPP_ERROR(get_logger(),
                "Target group '%s' joint '%s': invalid prismatic axis '%s' — skipping joint",
                group.c_str(), jname.c_str(), axis.c_str());
              continue;
            }
          } else {
            RCLCPP_ERROR(get_logger(),
              "Target group '%s' joint '%s': unknown type '%s' — skipping joint",
              group.c_str(), jname.c_str(), type.c_str());
            continue;
          }
          g.joints.push_back(tj);
        }

        if (g.joints.empty()) {
          RCLCPP_ERROR(get_logger(), "Target group '%s' has no usable joints — skipping",
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
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu tracking group(s) from rosparam",
      quest_tracked_groups.size());
  }

  // Cartesian groups: an end effector follows a controller's pose.
  if (has_param("control_cartesian.groups_name")) {
    get_param("control_cartesian.groups_name", quest_groups);
    for (const auto & group : quest_groups) {
      const bool is_arm = has_param("control_cartesian." + group +
          ".end_effector_frame_name");
      const bool is_hand = has_param("control_cartesian." + group + ".pose_action");

      if (is_arm) {
        QuestArmMap am{};
        am.group = group;
        get_param("control_cartesian." + group + ".end_effector_frame_name",
            am.end_effector_frame_name);
        get_param("control_cartesian." + group + ".target_frame_name", am.target_frame_name);
        get_param("control_cartesian." + group + ".motion_scale", am.motion_scale);
        get_param("control_cartesian." + group + ".enable_axis", am.enable_axis);
        am.arm_joint_trajectory_topic = group_trajectory_topic(group);
        if (am.arm_joint_trajectory_topic.empty()) {continue;}
        // Optional proximity thresholds — defaults are set in the struct
        if (has_param("control_cartesian." + group + ".proximity_threshold")) {
          get_param("control_cartesian." + group + ".proximity_threshold",
              am.proximity_threshold);
        }
        if (has_param("control_cartesian." + group + ".proximity_angle_threshold")) {
          get_param("control_cartesian." + group + ".proximity_angle_threshold",
              am.proximity_angle_threshold);
        }

        // Frames default from the controller side so older configs still work.
        get_param("control_cartesian." + group + ".controller_frame_name",
            am.controller_frame_name);
        get_param("control_cartesian." + group + ".controller_echo_frame_name",
            am.controller_echo_frame_name);
        // Log label only; the frames below carry the real identity.
        am.controller = group;

        if (am.controller_frame_name.empty() || am.controller_echo_frame_name.empty()) {
          RCLCPP_ERROR(get_logger(),
            "Target arm '%s' needs controller_frame_name and "
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

        if (has_param("control_cartesian." + group + ".pose_button")) {
          get_param("control_cartesian." + group + ".pose_button", hm.pose_button);
        }
        if (has_param("control_cartesian." + group + ".pose_open")) {
          get_param("control_cartesian." + group + ".pose_open", hm.pose_open);
          get_param("control_cartesian." + group + ".pose_close", hm.pose_close);
          get_param("control_cartesian." + group + ".pose_action", hm.pose_action);
        }
        quest_hand_mappings[group] = hm;
      } else {
        mark_visited("control_cartesian." + group);
        RCLCPP_WARN(get_logger(), "Cartesian group '%s' defines no end effector — skipping",
            group.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu cartesian arm and %zu hand parameters from rosparam",
      quest_arm_mappings.size(), quest_hand_mappings.size());
    has_cartesian_groups = !quest_groups.empty();

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

  requires_joint_states = !joint_mappings.empty() || has_cartesian_groups ||
    !quest_tracked_groups.empty();

  report_config_summary();

  if (requires_joint_states && joint_states_topic.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "joint_states_topic is required for joint or quest teleop, disabling those controls.");
    joint_mappings.clear();
    quest_arm_mappings.clear();
    quest_hand_mappings.clear();
    quest_groups.clear();
    has_cartesian_groups = false;
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
    "control_tracking.", "control_cartesian.", "robot_topic_name.", "base_max_speed."
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


bool SOBITSTeleop::send_pose(
  const PoseMap & pose_map, const std::vector<std::string> & exclude,
  const PoseJointGroup * only)
{
  // YAML-defined pose: publish straight to the controllers, no action server.
  if (!pose_map.joint_groups.empty()) {
    for (const auto & pg : pose_map.joint_groups) {
      if (only && &pg != only) {continue;}
      if (!exclude.empty() &&
        std::find(exclude.begin(), exclude.end(),
        topic_group_name(pg.joint_trajectory_topic)) != exclude.end())
      {
        continue;
      }
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
    // Shared stick: ignore a push that leans toward the guarded axis.
    if (m.dominant_over >= 0 &&
      std::abs(axis_val) <= std::abs(axis_value(m.dominant_over)))
    {
      continue;
    }

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
      if (button_pressed(pg.button)) {send_pose(pose_map, {}, &pg);}
    }
  }
}

// Steps a blend's joints toward whichever pose the input selects, at a speed
// set by how far it is pushed. Releasing leaves them where they are.
void SOBITSTeleop::process_pose_blends()
{
  for (const auto & bl : pose_blends) {
    const bool gated = bl.enable_axis >= 0 || bl.enable_button >= 0;
    if (gated && !axis_held(bl.enable_axis) && !button_down(bl.enable_button)) {continue;}

    double deflection = axis_value(bl.axis) * bl.axis_sign;
    if (button_down(bl.to_button)) {deflection = 1.0;}
    if (button_down(bl.from_button)) {deflection = -1.0;}
    if (std::abs(deflection) < 0.1) {continue;}

    const bool toward_to = deflection > 0.0;
    const double step = bl.speed * std::abs(deflection) * jog_tick_scale_;

    for (const auto & bg : bl.groups) {
      trajectory_msgs::msg::JointTrajectory traj;
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for (size_t i = 0; i < bg.joint_names.size(); ++i) {
        const std::string & jn = bg.joint_names[i];
        const double goal = toward_to ? bg.to_positions[i] : bg.from_positions[i];
        const double cur = joint_pos.count(jn) ? joint_pos.at(jn) : goal;
        double next = cur + ((goal > cur) ? step : -step);
        if ((goal > cur && next > goal) || (goal < cur && next < goal)) {next = goal;}
        if (!clamp_to_limits_checked(jn, next)) {continue;}
        joint_pos[jn] = next;
        traj.joint_names.push_back(jn);
        pt.positions.push_back(next);
      }
      if (traj.joint_names.empty()) {continue;}

      pt.time_from_start = rclcpp::Duration::from_seconds(dt());
      traj.points.push_back(pt);
      auto it = joint_pub.find(bg.joint_trajectory_topic);
      if (it != joint_pub.end() && it->second) {it->second->publish(traj);}
    }
  }
}

// Each press sends the next pose in the list, wrapping at the end.
void SOBITSTeleop::process_pose_cycles()
{
  for (auto & pc : pose_cycles) {
    if (!button_pressed(pc.button)) {continue;}
    // Debounce: the pose motion takes far longer than a button repeat.
    if (pc.last_press.nanoseconds() > 0 &&
      (this->now() - pc.last_press).seconds() < 0.4)
    {
      continue;
    }
    pc.last_press = this->now();

    const std::string & pose_name = pc.poses[pc.index];
    for (const auto & pm : pose_mappings) {
      if (pm.pose_name != pose_name) {continue;}
      send_pose(pm, pc.exclude_groups);
      break;
    }
    pc.index = (pc.index + 1) % pc.poses.size();
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
      const double linear_scale =
        cvm.linear_max * (fast_mode ? cvm.fast_linear_scale : cvm.linear_scale);
      const double angular_scale =
        cvm.angular_max * (fast_mode ? cvm.fast_angular_scale : cvm.angular_scale);

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

// Latch/track one control_tracking group. Trigger comes from /joy so releasing
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
  process_pose_blends();
  process_pose_cycles();
  process_cmd_vel();

  // Quest controllers: Unity publishes Quest frames directly under base_footprint.
  bool base_odom_ok = true;  // always ready; kept as guard variable for structure

  if (this->has_parameter("control_cartesian.groups_name")) {
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

  if (m.hand_pose_in_flight && this->now() >= m.hand_pose_deadline) {
    m.hand_pose_in_flight = false;
  }
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::SOBITSTeleop)
