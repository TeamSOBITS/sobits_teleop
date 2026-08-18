#include "sobits_teleop/servo_target_bridge.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>

namespace sobits_teleop
{

// Servo's command output rate (publish_period 0.02 s); sizes the escape history.
constexpr double kServoCmdRateHz = 50.0;

// ── Constructor ──

ServoTargetBridge::ServoTargetBridge(const rclcpp::NodeOptions & options)
: Node(
    "servo_target_bridge",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!this->has_parameter("servo_bridge.pose_rate_hz")) {
    this->declare_parameter("servo_bridge.pose_rate_hz", 100.0);
  }
  pose_rate_hz_ = this->get_parameter("servo_bridge.pose_rate_hz").as_double();

  if (!this->has_parameter("servo_bridge.arms")) {
    this->declare_parameter("servo_bridge.arms",
      std::vector<std::string>{"arm_right", "arm_left"});
  }

  auto arm_names = this->get_parameter("servo_bridge.arms").as_string_array();

  // Shared default for the per-arm reach clamp; per-arm max_reach overrides it.
  if (!this->has_parameter("servo_bridge.max_reach")) {
    this->declare_parameter("servo_bridge.max_reach", 0.0);
  }
  const double shared_max_reach =
    this->get_parameter("servo_bridge.max_reach").as_double();

  // Shared singularity-escape defaults; per-arm keys override.
  if (!this->has_parameter("servo_bridge.escape_step")) {
    this->declare_parameter("servo_bridge.escape_step", 0.0);
  }
  const double shared_escape_step =
    this->get_parameter("servo_bridge.escape_step").as_double();

  if (!this->has_parameter("servo_bridge.escape_timeout_s")) {
    this->declare_parameter("servo_bridge.escape_timeout_s", 2.0);
  }
  const double shared_escape_timeout =
    this->get_parameter("servo_bridge.escape_timeout_s").as_double();

  if (!this->has_parameter("servo_bridge.reset_on_halt")) {
    this->declare_parameter("servo_bridge.reset_on_halt", true);
  }
  const bool shared_reset_on_halt =
    this->get_parameter("servo_bridge.reset_on_halt").as_bool();

  if (!this->has_parameter("servo_bridge.reset_cooldown_s")) {
    this->declare_parameter("servo_bridge.reset_cooldown_s", 1.0);
  }
  const double shared_reset_cooldown =
    this->get_parameter("servo_bridge.reset_cooldown_s").as_double();

  if (!this->has_parameter("servo_bridge.joint_escape_time_s")) {
    this->declare_parameter("servo_bridge.joint_escape_time_s", 0.0);
  }
  const double shared_joint_escape_time =
    this->get_parameter("servo_bridge.joint_escape_time_s").as_double();

  for (const auto & arm_name : arm_names) {
    // Frames/topics default from the arm name (arm_right -> side "right");
    // the YAML only carries values that break the convention.
    const std::string side =
      arm_name.rfind("arm_", 0) == 0 ? arm_name.substr(4) : arm_name;

    auto tf_key = "servo_bridge." + arm_name + ".target_frame_name";
    auto bf_key = "servo_bridge." + arm_name + ".base_frame_name";
    auto ee_key = "servo_bridge." + arm_name + ".end_effector_frame_name";
    auto se_key = "servo_bridge." + arm_name + ".servo_ee_frame";
    auto sn_key = "servo_bridge." + arm_name + ".servo_node";
    auto en_key = "servo_bridge." + arm_name + ".enable_topic";
    auto ro_key = "servo_bridge." + arm_name + ".reach_origin_frame";
    auto mr_key = "servo_bridge." + arm_name + ".max_reach";
    auto es_key = "servo_bridge." + arm_name + ".escape_step";
    auto et_key = "servo_bridge." + arm_name + ".escape_timeout_s";
    auto st_key = "servo_bridge." + arm_name + ".status_topic";
    auto rh_key = "servo_bridge." + arm_name + ".reset_on_halt";
    auto rc_key = "servo_bridge." + arm_name + ".reset_cooldown_s";
    auto jt_key = "servo_bridge." + arm_name + ".joint_traj_topic";
    auto je_key = "servo_bridge." + arm_name + ".joint_escape_time_s";
    auto jl_key = "servo_bridge." + arm_name + ".joint_escape_lookback_s";

    if (!this->has_parameter(tf_key)) {
      this->declare_parameter(tf_key, side + "_target_link");
    }
    if (!this->has_parameter(bf_key)) {
      this->declare_parameter(bf_key, "base_footprint");
    }
    if (!this->has_parameter(ee_key)) {
      this->declare_parameter(ee_key, "hand_" + side + "_end_effector_link");
    }
    if (!this->has_parameter(se_key)) {
      this->declare_parameter(se_key, std::string(""));
    }
    if (!this->has_parameter(sn_key)) {
      this->declare_parameter(sn_key, std::string("servo_") + arm_name);
    }
    if (!this->has_parameter(en_key)) {
      this->declare_parameter(en_key, arm_name + "/moveit_track_enabled");
    }
    if (!this->has_parameter(ro_key)) {
      this->declare_parameter(ro_key, arm_name + "_shoulder_tilt_link");
    }
    if (!this->has_parameter(mr_key)) {
      this->declare_parameter(mr_key, shared_max_reach);
    }
    if (!this->has_parameter(es_key)) {
      this->declare_parameter(es_key, shared_escape_step);
    }
    if (!this->has_parameter(et_key)) {
      this->declare_parameter(et_key, shared_escape_timeout);
    }
    if (!this->has_parameter(rh_key)) {
      this->declare_parameter(rh_key, shared_reset_on_halt);
    }
    if (!this->has_parameter(rc_key)) {
      this->declare_parameter(rc_key, shared_reset_cooldown);
    }
    // Same controller topic servo commands, so the escape reaches the same JTC.
    if (!this->has_parameter(jt_key)) {
      this->declare_parameter(jt_key, arm_name + "_position_controller/joint_trajectory");
    }
    if (!this->has_parameter(je_key)) {
      this->declare_parameter(je_key, shared_joint_escape_time);
    }
    if (!this->has_parameter(jl_key)) {
      this->declare_parameter(jl_key, 1.0);
    }
    // servo publishes status on "~/status" relative to its own node name.
    if (!this->has_parameter(st_key)) {
      this->declare_parameter(st_key,
        this->get_parameter(sn_key).as_string() + "/status");
    }

    ServoBridgeArmConfig cfg;
    cfg.target_frame = this->get_parameter(tf_key).as_string();
    cfg.base_frame = this->get_parameter(bf_key).as_string();
    cfg.end_effector_frame = this->get_parameter(ee_key).as_string();
    cfg.servo_ee_frame = this->get_parameter(se_key).as_string();
    cfg.servo_node = this->get_parameter(sn_key).as_string();
    cfg.enable_topic = this->get_parameter(en_key).as_string();
    cfg.reach_origin_frame = this->get_parameter(ro_key).as_string();
    cfg.max_reach = this->get_parameter(mr_key).as_double();
    cfg.escape_step = this->get_parameter(es_key).as_double();
    cfg.escape_timeout_s = this->get_parameter(et_key).as_double();
    cfg.reset_on_halt = this->get_parameter(rh_key).as_bool();
    cfg.reset_cooldown_s = this->get_parameter(rc_key).as_double();
    cfg.joint_traj_topic = this->get_parameter(jt_key).as_string();
    cfg.joint_escape_time_s = this->get_parameter(je_key).as_double();
    cfg.joint_escape_lookback_s = this->get_parameter(jl_key).as_double();
    const std::string status_topic = this->get_parameter(st_key).as_string();

    auto arm_data = std::make_unique<ArmBridgeData>();
    arm_data->config = cfg;

    // Pose publisher: relative topic under the servo node's own namespace,
    // e.g. "servo_arm_right/pose_target_cmds" -> resolves under /sobit_home.
    arm_data->pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      cfg.servo_node + "/pose_target_cmds", rclcpp::SystemDefaultsQoS());

    arm_data->switch_command_type_client =
      this->create_client<ServoCommandType>(cfg.servo_node + "/switch_command_type");
    arm_data->pause_servo_client =
      this->create_client<SetBool>(cfg.servo_node + "/pause_servo");

    // reliable + transient_local so a late-starting bridge still receives the
    // current enable state (publisher side matches).
    rclcpp::QoS enable_qos(1);
    enable_qos.reliable();
    enable_qos.transient_local();

    arm_data->enable_sub = this->create_subscription<std_msgs::msg::Bool>(
      cfg.enable_topic, enable_qos,
      [this, arm_name](const std_msgs::msg::Bool::SharedPtr msg) {
        enable_callback(arm_name, msg);
      });

    arm_data->joint_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      cfg.joint_traj_topic, rclcpp::SystemDefaultsQoS());

    // Snoop servo's command stream to learn the group's joint names and their
    // last healthy positions; the bridge has no robot model of its own.
    arm_data->servo_cmd_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      cfg.joint_traj_topic, rclcpp::QoS(1),
      [this, arm_name](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        servo_cmd_callback(arm_name, msg);
      });

    // Must be RELIABLE to match servo_node's status publisher; depth 1 since
    // only the latest code matters.
    arm_data->status_sub = this->create_subscription<moveit_msgs::msg::ServoStatus>(
      status_topic, rclcpp::QoS(1).reliable(),
      [this, arm_name](const moveit_msgs::msg::ServoStatus::SharedPtr msg) {
        status_callback(arm_name, msg);
      });

    arms_[arm_name] = std::move(arm_data);

    RCLCPP_INFO(get_logger(),
      "Arm '%s': servo_node='%s', target_frame_name='%s', base_frame_name='%s', "
      "end_effector_frame_name='%s', servo_ee_frame='%s', enable_topic='%s', "
      "reach_origin_frame='%s', max_reach=%.3f",
      arm_name.c_str(), cfg.servo_node.c_str(), cfg.target_frame.c_str(),
      cfg.base_frame.c_str(), cfg.end_effector_frame.c_str(),
      cfg.servo_ee_frame.c_str(), cfg.enable_topic.c_str(),
      cfg.reach_origin_frame.c_str(), cfg.max_reach);

    // Startup: switch_command_type(POSE) + pause_servo(true) per arm, async and
    // retried on a slow timer (servo_node's services come up concurrently).
    arms_[arm_name]->startup_retry_timer = this->create_wall_timer(
      std::chrono::seconds(2),
      [this, arm_name]() {try_startup_sequence(arm_name);});
    // Fire once immediately too, in case services are already up.
    try_startup_sequence(arm_name);
  }

  // One shared timer drives all arms' pose publishing at pose_rate_hz_.
  auto period = std::chrono::duration<double>(1.0 / pose_rate_hz_);
  pose_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {pose_timer_callback();});

  RCLCPP_INFO(get_logger(),
    "ServoTargetBridge: %zu arm(s), pose_rate=%.1f Hz",
    arms_.size(), pose_rate_hz_);
}

// ── Startup sequence: async POSE switch + pause, retried until both succeed ──

void ServoTargetBridge::try_startup_sequence(const std::string & arm_name)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  try_send_pause(arm_name);

  if (arm.command_type_set.load() && arm.initial_pause_set.load() &&
    arm.pending_pause.load() == -1)
  {
    if (arm.startup_retry_timer) {
      arm.startup_retry_timer->cancel();
    }
    return;
  }

  if (!arm.command_type_set.load()) {
    if (arm.switch_command_type_client->service_is_ready()) {
      auto req = std::make_shared<ServoCommandType::Request>();
      req->command_type = ServoCommandType::Request::POSE;
      arm.switch_command_type_client->async_send_request(
        req,
        [this, arm_name](rclcpp::Client<ServoCommandType>::SharedFuture future) {
          auto it2 = arms_.find(arm_name);
          if (it2 == arms_.end()) {return;}
          auto & arm2 = *it2->second;
          auto resp = future.get();
          if (resp->success) {
            arm2.command_type_set = true;
            RCLCPP_INFO(get_logger(),
              "Arm '%s': switch_command_type(POSE) succeeded on '%s'",
              arm_name.c_str(), arm2.config.servo_node.c_str());
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Arm '%s': switch_command_type(POSE) rejected — will retry",
              arm_name.c_str());
          }
        });
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Arm '%s': switch_command_type service not available yet on '%s' — retrying",
        arm_name.c_str(), arm.config.servo_node.c_str());
    }
  }

  // Once the operator has enabled, the enable path owns pause state and the
  // startup pause is moot — never re-pause an arm the operator believes is on.
  if (arm.enabled.load()) {
    arm.initial_pause_set = true;
  }

  if (!arm.initial_pause_set.load()) {
    if (arm.pause_servo_client->service_is_ready()) {
      auto req = std::make_shared<SetBool::Request>();
      req->data = true;
      arm.pause_servo_client->async_send_request(
        req,
        [this, arm_name](rclcpp::Client<SetBool>::SharedFuture future) {
          auto it2 = arms_.find(arm_name);
          if (it2 == arms_.end()) {return;}
          auto & arm2 = *it2->second;
          auto resp = future.get();
          if (resp->success) {
            arm2.initial_pause_set = true;
            RCLCPP_INFO(get_logger(),
              "Arm '%s': initial pause_servo(true) succeeded on '%s'",
              arm_name.c_str(), arm2.config.servo_node.c_str());
            // Race guard: an enable arrived while this startup pause was in flight —
            // undo immediately with an async unpause.
            if (arm2.enabled.load()) {
              RCLCPP_WARN(get_logger(),
                "Arm '%s': startup pause_servo(true) landed after an enable — "
                "re-unpausing immediately", arm_name.c_str());
              auto unpause_req = std::make_shared<SetBool::Request>();
              unpause_req->data = false;
              arm2.pause_servo_client->async_send_request(
                unpause_req,
                [this, arm_name](rclcpp::Client<SetBool>::SharedFuture unpause_future) {
                  auto resp2 = unpause_future.get();
                  RCLCPP_INFO(get_logger(),
                    "Arm '%s': re-unpause after startup-pause race -> %s",
                    arm_name.c_str(), resp2->success ? "ok" : "FAILED");
                });
            }
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Arm '%s': initial pause_servo(true) rejected — will retry",
              arm_name.c_str());
          }
        });
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Arm '%s': pause_servo service not available yet on '%s' — retrying",
        arm_name.c_str(), arm.config.servo_node.c_str());
    }
  }
}

// ── Send the pending pause/unpause, retried by the caller until confirmed ──

void ServoTargetBridge::try_send_pause(const std::string & arm_name)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  const int wanted = arm.pending_pause.load();
  if (wanted == -1) {return;}

  if (!arm.pause_servo_client->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Arm '%s': pause_servo service not ready — servo stays %s until it returns",
      arm_name.c_str(), wanted == 1 ? "paused" : "unpaused");
    return;
  }

  auto req = std::make_shared<SetBool::Request>();
  req->data = (wanted == 1);
  arm.pause_servo_client->async_send_request(
    req,
    [this, arm_name, wanted](rclcpp::Client<SetBool>::SharedFuture future) {
      auto it2 = arms_.find(arm_name);
      if (it2 == arms_.end()) {return;}
      auto & arm2 = *it2->second;
      auto resp = future.get();
      if (resp->success) {
        // Only clear if no newer toggle arrived while this request was in flight.
        int expected = wanted;
        if (arm2.pending_pause.compare_exchange_strong(expected, -1)) {
          RCLCPP_INFO(get_logger(), "Arm '%s': pause_servo(%s) -> ok",
            arm_name.c_str(), wanted == 1 ? "true" : "false");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Arm '%s': pause_servo(%s) rejected — will retry",
          arm_name.c_str(), wanted == 1 ? "true" : "false");
      }
    });
}

// ── Enable / disable callback (async only — never blocks the executor) ──

void ServoTargetBridge::enable_callback(
  const std::string & arm_name,
  const std_msgs::msg::Bool::SharedPtr msg)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  if (msg->data) {
    if (arm.enabled.load()) {return;}

    // No waiting on responses — Servo tolerates early pose commands while paused.
    // Re-request POSE mode in case servo_node restarted, then unpause.
    if (arm.switch_command_type_client->service_is_ready()) {
      auto req = std::make_shared<ServoCommandType::Request>();
      req->command_type = ServoCommandType::Request::POSE;
      arm.switch_command_type_client->async_send_request(
        req,
        [this, arm_name](rclcpp::Client<ServoCommandType>::SharedFuture future) {
          auto resp = future.get();
          RCLCPP_INFO(get_logger(), "Arm '%s': switch_command_type(POSE) on enable -> %s",
            arm_name.c_str(), resp->success ? "ok" : "FAILED");
        });
    } else {
      RCLCPP_WARN(get_logger(),
        "Arm '%s': switch_command_type service not ready on enable — pose "
        "commands will be published anyway (Servo may drop them until it is)",
        arm_name.c_str());
    }

    // Unpause is retried until confirmed, so it's never silently skipped.
    arm.pending_pause = 0;
    try_send_pause(arm_name);
    if (arm.startup_retry_timer) {arm.startup_retry_timer->reset();}

    arm.enabled = true;
    RCLCPP_INFO(get_logger(), "Servo tracking ENABLED for '%s'", arm_name.c_str());
  } else {
    if (!arm.enabled.load()) {return;}
    arm.enabled = false;

    // Drop the escape anchor: the next latch starts from a new EE pose, and a
    // stale one would drag the arm to wherever it was healthy last session.
    arm.have_last_good = false;
    arm.escape_gave_up = false;
    arm.escaping = false;
    arm.reset_in_flight = false;
    arm.joint_history.clear();
    arm.have_escape_joints = false;

    arm.pending_pause = 1;
    try_send_pause(arm_name);
    if (arm.startup_retry_timer) {arm.startup_retry_timer->reset();}

    RCLCPP_INFO(get_logger(), "Servo tracking DISABLED for '%s'", arm_name.c_str());
  }
}

// ── Servo status: latch/clear the singularity halt ──

void ServoTargetBridge::status_callback(
  const std::string & arm_name,
  const moveit_msgs::msg::ServoStatus::SharedPtr msg)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  const bool halted = (msg->code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY);
  const bool was_halted = arm.singularity_halt.exchange(halted);

  if (halted && !was_halted) {
    arm.halt_start = this->now();
    arm.escape_gave_up = false;
    RCLCPP_WARN(get_logger(),
      "Arm '%s': HALT_FOR_SINGULARITY — %s", arm_name.c_str(),
      arm.config.reset_on_halt ? "resetting servo" : "reset DISABLED");
  } else if (!halted && was_halted) {
    // Re-arm: a later halt gets a fresh escape budget rather than inheriting
    // the previous one's give-up state.
    arm.escape_gave_up = false;
    RCLCPP_INFO(get_logger(), "Arm '%s': singularity halt cleared", arm_name.c_str());
  }

  // Servo ignores pose commands while halted, so retargeting alone cannot
  // recover; the latched state has to be cleared first.
  if (halted && arm.config.reset_on_halt && arm.enabled.load()) {
    reset_after_halt(arm_name);
  }
}

// ── Cache the group's joint configuration from servo's command stream ──

void ServoTargetBridge::servo_cmd_callback(
  const std::string & arm_name,
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  // Only healthy commands are worth escaping to.
  if (arm.singularity_halt.load() || msg->points.empty()) {return;}
  if (msg->points.front().positions.size() != msg->joint_names.size()) {return;}

  arm.escape_joint_names = msg->joint_names;

  // Keep a short history: the newest healthy command sits right next to the
  // singularity, so escaping to it lands straight back in the halt.
  arm.joint_history.push_back(msg->points.front().positions);
  const size_t depth = static_cast<size_t>(
    std::max(1.0, arm.config.joint_escape_lookback_s * kServoCmdRateHz));
  while (arm.joint_history.size() > depth) {
    arm.joint_history.pop_front();
  }
  // The oldest retained sample is the furthest back from the trap.
  arm.escape_joint_positions = arm.joint_history.front();
  arm.have_escape_joints = true;
}

// ── Jointspace escape: replay the last healthy configuration ──

void ServoTargetBridge::send_joint_escape(const std::string & arm_name)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  if (!arm.have_escape_joints || arm.config.joint_escape_time_s <= 0.0) {return;}

  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  traj.joint_names = arm.escape_joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = arm.escape_joint_positions;
  pt.velocities.assign(arm.escape_joint_positions.size(), 0.0);
  pt.time_from_start = rclcpp::Duration::from_seconds(arm.config.joint_escape_time_s);
  traj.points.push_back(pt);

  arm.joint_traj_pub->publish(traj);
  RCLCPP_WARN(get_logger(),
    "Arm '%s': jointspace escape — replaying last healthy configuration over %.1f s",
    arm_name.c_str(), arm.config.joint_escape_time_s);
}

// ── Clear a latched halt: pause(true) then pause(false) ──

void ServoTargetBridge::reset_after_halt(const std::string & arm_name)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {return;}
  auto & arm = *it->second;

  if (arm.reset_in_flight.load()) {return;}
  if (!arm.pause_servo_client->service_is_ready()) {return;}

  // Cooldown: status streams at the servo loop rate, so an uncooled reset would
  // fire hundreds of times per second.
  const rclcpp::Time now = this->now();
  if (arm.last_reset.nanoseconds() > 0 &&
    (now - arm.last_reset).seconds() < arm.config.reset_cooldown_s)
  {
    return;
  }
  arm.last_reset = now;
  arm.reset_in_flight = true;

  auto pause_req = std::make_shared<SetBool::Request>();
  pause_req->data = true;
  arm.pause_servo_client->async_send_request(
    pause_req,
    [this, arm_name](rclcpp::Client<SetBool>::SharedFuture pause_future) {
      auto it2 = arms_.find(arm_name);
      if (it2 == arms_.end()) {return;}
      auto & arm2 = *it2->second;

      if (!pause_future.get()->success) {
        RCLCPP_WARN(get_logger(), "Arm '%s': halt-reset pause(true) rejected",
          arm_name.c_str());
        arm2.reset_in_flight = false;
        return;
      }
      // Grip may have been released while the pause was in flight — leave the
      // arm paused in that case, which is what a disable wants anyway.
      if (!arm2.enabled.load()) {
        arm2.reset_in_flight = false;
        return;
      }

      // Servo is paused and no longer owns the controller: drive the arm out in
      // jointspace, which needs no Jacobian and so is immune to the singularity.
      send_joint_escape(arm_name);

      // Resume only after the escape motion completes, else servo's first tick
      // re-halts at the singular pose and fights the trajectory.
      const double settle =
      arm2.have_escape_joints ? arm2.config.joint_escape_time_s + 0.2 : 0.0;
      arm2.resume_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(settle)),
        [this, arm_name]() {
          auto it3 = arms_.find(arm_name);
          if (it3 == arms_.end()) {return;}
          auto & arm3 = *it3->second;
          arm3.resume_timer->cancel();

          if (!arm3.enabled.load()) {
            arm3.reset_in_flight = false;
            return;
          }
          auto unpause_req = std::make_shared<SetBool::Request>();
          unpause_req->data = false;
          arm3.pause_servo_client->async_send_request(
            unpause_req,
            [this, arm_name](rclcpp::Client<SetBool>::SharedFuture unpause_future) {
              auto it4 = arms_.find(arm_name);
              if (it4 == arms_.end()) {return;}
              auto & arm4 = *it4->second;
              const bool ok = unpause_future.get()->success;
              RCLCPP_WARN(get_logger(), "Arm '%s': halt-reset pause cycle -> %s",
                arm_name.c_str(), ok ? "servo resumed" : "unpause FAILED");
              arm4.reset_in_flight = false;
            });
        });
    });
}

// ── Shared pose-publish timer ──

void ServoTargetBridge::pose_timer_callback()
{
  for (auto & [arm_name, arm_ptr] : arms_) {
    auto & arm = *arm_ptr;
    if (!arm.enabled.load()) {continue;}

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        arm.config.base_frame, arm.config.target_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Arm '%s': TF lookup '%s' -> '%s' failed: %s — skipping tick",
        arm_name.c_str(), arm.config.base_frame.c_str(),
        arm.config.target_frame.c_str(), e.what());
      continue;
    }

    // T(base -> desired EE pose): the target TF describes where the EE should be.
    tf2::Transform base_to_target(
      tf2::Quaternion(
        tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y,
        tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w),
      tf2::Vector3(
        tf_stamped.transform.translation.x, tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z));

    // Reach clamp: pull out-of-reach targets onto the max_reach sphere —
    // full extension is an elbow singularity that latches a servo e-stop.
    if (arm.config.max_reach > 0.0 && !arm.config.reach_origin_frame.empty()) {
      try {
        auto sh = tf_buffer_->lookupTransform(
          arm.config.base_frame, arm.config.reach_origin_frame, tf2::TimePointZero);
        tf2::Vector3 shoulder(
          sh.transform.translation.x, sh.transform.translation.y,
          sh.transform.translation.z);
        tf2::Vector3 offset = base_to_target.getOrigin() - shoulder;
        const double dist = offset.length();
        if (dist > arm.config.max_reach) {
          base_to_target.setOrigin(
            shoulder + offset * (arm.config.max_reach / dist));
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Arm '%s': target %.2f m from shoulder — clamped to %.2f m",
            arm_name.c_str(), dist, arm.config.max_reach);
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Arm '%s': shoulder TF '%s' unavailable (%s) — reach clamp skipped",
          arm_name.c_str(), arm.config.reach_origin_frame.c_str(), e.what());
      }
    }

    // Singularity escape: while halted, ignore the operator target and walk the
    // command back toward the last healthy EE pose — the trap pose only re-halts.
    if (arm.singularity_halt.load() && arm.config.escape_step > 0.0 &&
      arm.have_last_good && !arm.escape_gave_up)
    {
      const double held = (this->now() - arm.halt_start).seconds();
      if (arm.config.escape_timeout_s > 0.0 && held > arm.config.escape_timeout_s) {
        arm.escape_gave_up = true;
        RCLCPP_ERROR(get_logger(),
          "Arm '%s': still halted after %.1f s — escape abandoned, release grip "
          "and re-latch away from the singularity", arm_name.c_str(), held);
      } else {
        // Step from the previous command, not the operator target: the operator
        // keeps pushing into the trap, so stepping from it never converges.
        if (!arm.escaping) {
          arm.escape_cmd = base_to_target;
          arm.escaping = true;
        }
        const tf2::Vector3 to_good =
          arm.last_good_cmd.getOrigin() - arm.escape_cmd.getOrigin();
        const double dist = to_good.length();
        const double frac = (dist > arm.config.escape_step) ?
          (arm.config.escape_step / dist) : 1.0;

        arm.escape_cmd.setOrigin(arm.escape_cmd.getOrigin() + to_good * frac);
        arm.escape_cmd.setRotation(
          arm.escape_cmd.getRotation().slerp(
            arm.last_good_cmd.getRotation(), frac).normalized());
        base_to_target = arm.escape_cmd;

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Arm '%s': escaping singularity — %.3f m to last healthy pose",
          arm_name.c_str(), dist);
      }
    } else if (!arm.singularity_halt.load()) {
      arm.escaping = false;
      // Anchor on the actual EE pose, not the commanded target: the target may
      // already sit in the trap when the halt lands, making it useless to escape to.
      try {
        auto ee = tf_buffer_->lookupTransform(
          arm.config.base_frame, arm.config.end_effector_frame, tf2::TimePointZero);
        tf2::fromMsg(ee.transform, arm.last_good_cmd);
        arm.have_last_good = true;
      } catch (const tf2::TransformException &) {
        // Keep the previous anchor; a stale healthy pose beats none.
      }
    }

    // Re-express the target for the servo-driven frame when it differs from the EE:
    // T(base->cmd) = T(base->target) * T(ee->servo_ee). Looked up per tick, not cached.
    tf2::Transform base_to_cmd = base_to_target;
    if (!arm.config.servo_ee_frame.empty() &&
      !arm.config.end_effector_frame.empty() &&
      arm.config.servo_ee_frame != arm.config.end_effector_frame)
    {
      try {
        auto ee_se = tf_buffer_->lookupTransform(
          arm.config.end_effector_frame, arm.config.servo_ee_frame,
          tf2::TimePointZero);
        base_to_cmd = base_to_target * tf2::Transform(
          tf2::Quaternion(
            ee_se.transform.rotation.x, ee_se.transform.rotation.y,
            ee_se.transform.rotation.z, ee_se.transform.rotation.w),
          tf2::Vector3(
            ee_se.transform.translation.x, ee_se.transform.translation.y,
            ee_se.transform.translation.z));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Arm '%s': EE->servo_ee TF '%s'->'%s' unavailable (%s) — skipping tick",
          arm_name.c_str(), arm.config.end_effector_frame.c_str(),
          arm.config.servo_ee_frame.c_str(), e.what());
        continue;
      }
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = arm.config.base_frame;
    pose_msg.header.stamp = this->now();
    pose_msg.pose.position.x = base_to_cmd.getOrigin().x();
    pose_msg.pose.position.y = base_to_cmd.getOrigin().y();
    pose_msg.pose.position.z = base_to_cmd.getOrigin().z();
    pose_msg.pose.orientation = tf2::toMsg(base_to_cmd.getRotation());

    arm.pose_pub->publish(pose_msg);
  }
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::ServoTargetBridge)
