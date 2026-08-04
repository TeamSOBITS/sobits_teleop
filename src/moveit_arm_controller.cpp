#include "sobits_teleop/moveit_arm_controller.hpp"
#include <rclcpp/parameter_client.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>

namespace sobits_teleop
{

// ── Constructor ─────────────────────────────────────────────────

MoveitArmController::MoveitArmController(const rclcpp::NodeOptions & options)
: Node(
    "moveit_arm_controller",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!this->has_parameter("arm_teleop.update_rate_hz"))
    this->declare_parameter("arm_teleop.update_rate_hz", 15.0);
  if (!this->has_parameter("arm_teleop.max_cartesian_step_m"))
    this->declare_parameter("arm_teleop.max_cartesian_step_m", 0.03);
  if (!this->has_parameter("arm_teleop.min_cartesian_fraction"))
    this->declare_parameter("arm_teleop.min_cartesian_fraction", 0.5);
  if (!this->has_parameter("arm_teleop.arrival_threshold_m"))
    this->declare_parameter("arm_teleop.arrival_threshold_m", 0.01);
  if (!this->has_parameter("arm_teleop.velocity_scaling"))
    this->declare_parameter("arm_teleop.velocity_scaling", 0.3);
  if (!this->has_parameter("arm_teleop.acceleration_scaling"))
    this->declare_parameter("arm_teleop.acceleration_scaling", 0.3);
  if (!this->has_parameter("arm_teleop.eef_step_m"))
    this->declare_parameter("arm_teleop.eef_step_m", 0.03);
  if (!this->has_parameter("arm_teleop.replan_threshold_m"))
    this->declare_parameter("arm_teleop.replan_threshold_m", 0.02);
  if (!this->has_parameter("arm_teleop.traj_lookahead_ms"))
    this->declare_parameter("arm_teleop.traj_lookahead_ms", 50);
  if (!this->has_parameter("arm_teleop.ompl_planning_timeout_s"))
    this->declare_parameter("arm_teleop.ompl_planning_timeout_s", 0.5);
  if (!this->has_parameter("arm_teleop.preempt_threshold_m"))
    this->declare_parameter("arm_teleop.preempt_threshold_m", 0.15);
  if (!this->has_parameter("arm_teleop.arrival_threshold_rad"))
    this->declare_parameter("arm_teleop.arrival_threshold_rad", 0.05);   // ~3 deg
  if (!this->has_parameter("arm_teleop.replan_threshold_rad"))
    this->declare_parameter("arm_teleop.replan_threshold_rad", 0.05);    // ~3 deg
  if (!this->has_parameter("arm_teleop.preempt_threshold_rad"))
    this->declare_parameter("arm_teleop.preempt_threshold_rad", 0.26);   // ~15 deg
  if (!this->has_parameter("arm_teleop.avoid_collisions"))
    this->declare_parameter("arm_teleop.avoid_collisions", false);
  if (!this->has_parameter("arm_teleop.preempt_settle_ms"))
    this->declare_parameter("arm_teleop.preempt_settle_ms", 10);
  if (!this->has_parameter("arm_teleop.publish_mode"))
    this->declare_parameter("arm_teleop.publish_mode", std::string("topic"));

  update_rate_hz_           = this->get_parameter("arm_teleop.update_rate_hz").as_double();
  max_cartesian_step_m_     = this->get_parameter("arm_teleop.max_cartesian_step_m").as_double();
  min_cartesian_fraction_   = this->get_parameter("arm_teleop.min_cartesian_fraction").as_double();
  arrival_threshold_m_      = this->get_parameter("arm_teleop.arrival_threshold_m").as_double();
  velocity_scaling_         = this->get_parameter("arm_teleop.velocity_scaling").as_double();
  acceleration_scaling_     = this->get_parameter("arm_teleop.acceleration_scaling").as_double();
  eef_step_m_               = this->get_parameter("arm_teleop.eef_step_m").as_double();
  replan_threshold_m_       = this->get_parameter("arm_teleop.replan_threshold_m").as_double();
  traj_lookahead_ms_        = this->get_parameter("arm_teleop.traj_lookahead_ms").as_int();
  ompl_planning_timeout_s_  = this->get_parameter("arm_teleop.ompl_planning_timeout_s").as_double();
  preempt_threshold_m_      = this->get_parameter("arm_teleop.preempt_threshold_m").as_double();
  arrival_threshold_rad_    = this->get_parameter("arm_teleop.arrival_threshold_rad").as_double();
  replan_threshold_rad_     = this->get_parameter("arm_teleop.replan_threshold_rad").as_double();
  preempt_threshold_rad_    = this->get_parameter("arm_teleop.preempt_threshold_rad").as_double();
  avoid_collisions_         = this->get_parameter("arm_teleop.avoid_collisions").as_bool();
  preempt_settle_ms_        = this->get_parameter("arm_teleop.preempt_settle_ms").as_int();
  use_topic_                = (this->get_parameter("arm_teleop.publish_mode").as_string() == "topic");

  if (!this->has_parameter("arm_teleop.arms"))
    this->declare_parameter("arm_teleop.arms",
      std::vector<std::string>{"arm_left", "arm_right"});

  auto arm_names = this->get_parameter("arm_teleop.arms").as_string_array();

  for (const auto & arm_name : arm_names) {
    auto pg_key   = "arm_teleop." + arm_name + ".planning_group";
    auto tf_key   = "arm_teleop." + arm_name + ".target_frame";
    auto bf_key   = "arm_teleop." + arm_name + ".base_frame";
    auto traj_key = "arm_teleop." + arm_name + ".trajectory_topic";

    if (!this->has_parameter(pg_key))
      this->declare_parameter(pg_key, arm_name);
    if (!this->has_parameter(tf_key))
      this->declare_parameter(tf_key, arm_name + "_target_link");
    if (!this->has_parameter(bf_key))
      this->declare_parameter(bf_key, "base_footprint");
    if (!this->has_parameter(traj_key))
      this->declare_parameter(traj_key, arm_name + "_position_controller/joint_trajectory");

    ArmTeleopConfig cfg;
    cfg.planning_group    = this->get_parameter(pg_key).as_string();
    cfg.target_frame      = this->get_parameter(tf_key).as_string();
    cfg.base_frame        = this->get_parameter(bf_key).as_string();
    cfg.trajectory_topic  = this->get_parameter(traj_key).as_string();

    auto arm_data = std::make_unique<ArmData>();
    arm_data->config = cfg;

    // Derive action server name from the trajectory topic:
    // ".../joint_trajectory" → ".../follow_joint_trajectory"
    std::string action_topic = cfg.trajectory_topic;
    auto pos = action_topic.rfind('/');
    if (pos != std::string::npos) {
      action_topic = action_topic.substr(0, pos) + "/follow_joint_trajectory";
    }
    arm_data->action_client =
      rclcpp_action::create_client<FollowJointTrajectory>(this, action_topic);

    // Direct-publish path (publish_mode: topic): publishing a new trajectory
    // replaces the active one mid-flight (no goal/cancel handshake).
    arm_data->traj_pub =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        cfg.trajectory_topic, rclcpp::QoS(10).reliable());

    arms_[arm_name] = std::move(arm_data);

    auto sub = this->create_subscription<std_msgs::msg::Bool>(
      arm_name + "/moveit_track_enabled",
      rclcpp::QoS(1),
      [this, arm_name](const std_msgs::msg::Bool::SharedPtr msg) {
        enable_callback(arm_name, msg);
      });
    enable_subs_.push_back(sub);

    RCLCPP_INFO(get_logger(),
      "Arm '%s': target_frame='%s', base_frame='%s', action='%s'",
      arm_name.c_str(), cfg.target_frame.c_str(),
      cfg.base_frame.c_str(), action_topic.c_str());
  }

  RCLCPP_INFO(get_logger(),
    "MoveitArmController: rate=%.1f Hz, max_step=%.3f m, eef_step=%.3f m, "
    "replan_thresh=%.3f m, preempt_thresh=%.3f m, vel_scale=%.2f, "
    "lookahead=%d ms, ompl_timeout=%.2f s, avoid_collisions=%s, settle=%d ms, "
    "publish_mode=%s",
    update_rate_hz_, max_cartesian_step_m_, eef_step_m_,
    replan_threshold_m_, preempt_threshold_m_, velocity_scaling_,
    traj_lookahead_ms_, ompl_planning_timeout_s_,
    avoid_collisions_ ? "true" : "false", preempt_settle_ms_,
    use_topic_ ? "topic" : "action");

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state_callback(msg);
    });

  init_thread_ = std::thread([this]() { init_move_groups(); });
}

// ── Destructor ──────────────────────────────────────────────────

MoveitArmController::~MoveitArmController()
{
  for (auto & [name, arm] : arms_) {
    arm->enabled = false;
    cancel_trajectory(*arm);
  }
  for (auto & [name, arm] : arms_) {
    if (arm->thread.joinable()) {
      arm->thread.join();
    }
  }
  if (init_thread_.joinable()) {
    init_thread_.join();
  }
}

// ── MoveGroupInterface initialisation (background thread) ───────

void MoveitArmController::init_move_groups()
{
  std::string ns = this->get_namespace();
  if (!ns.empty() && ns.front() == '/') {
    ns = ns.substr(1);
  }

  if (!this->has_parameter("robot_description")) {
    RCLCPP_INFO(get_logger(),
      "Fetching robot_description from /%s/move_group ...", ns.c_str());

    auto tmp_node = std::make_shared<rclcpp::Node>(
      "moveit_arm_controller_param_fetch", this->get_namespace());
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      tmp_node, "/" + ns + "/move_group");

    if (!param_client->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(),
        "move_group parameter service not available — arm controller will be inactive");
      return;
    }

    auto base_params = param_client->get_parameters(
      {"robot_description", "robot_description_semantic"});

    if (base_params.empty() ||
        base_params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        base_params[0].as_string().empty()) {
      RCLCPP_ERROR(get_logger(),
        "robot_description is empty — arm controller will be inactive");
      return;
    }

    this->declare_parameter("robot_description", base_params[0].as_string());
    if (base_params.size() >= 2 &&
        base_params[1].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      this->declare_parameter("robot_description_semantic", base_params[1].as_string());
    }
    RCLCPP_INFO(get_logger(),
      "robot_description declared (%zu chars)", base_params[0].as_string().size());

    rcl_interfaces::msg::ListParametersResult all_param_names;
    constexpr uint64_t DEPTH_RECURSIVE = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
      all_param_names = param_client->list_parameters({"robot_description_planning"}, DEPTH_RECURSIVE);
      if (!all_param_names.names.empty()) break;
      RCLCPP_INFO(get_logger(),
        "Waiting for robot_description_planning params on move_group (attempt %d/20)...",
        attempt + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(get_logger(),
      "robot_description_planning: found %zu param names via list_parameters",
      all_param_names.names.size());

    if (!all_param_names.names.empty()) {
      const auto & names = all_param_names.names;
      size_t copied = 0;
      for (const auto & pname : names) {
        auto result = param_client->get_parameters({pname});
        if (result.empty()) continue;
        const auto & p = result[0];
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
        try {
          if (!this->has_parameter(p.get_name())) {
            this->declare_parameter(p.get_name(), p.get_parameter_value());
            ++copied;
          }
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        } catch (const std::exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "declare_parameter failed for '%s': %s", p.get_name().c_str(), e.what());
        }
      }
      RCLCPP_INFO(get_logger(),
        "robot_description_planning: copied %zu / %zu sub-parameters from move_group",
        copied, names.size());
    } else {
      RCLCPP_WARN(get_logger(),
        "robot_description_planning namespace empty on move_group — TOTG may fail");
    }
  }

  for (auto & [arm_name, arm_data] : arms_) {
    if (arm_data->mgi_ready.load()) continue;
    try {
      moveit::planning_interface::MoveGroupInterface::Options opts(
        arm_data->config.planning_group,
        "robot_description",
        "/" + ns);

      auto mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        opts,
        tf_buffer_,
        rclcpp::Duration::from_seconds(30.0));

      mgi->setMaxVelocityScalingFactor(velocity_scaling_);
      mgi->setMaxAccelerationScalingFactor(acceleration_scaling_);

      if (mgi->getJoints().empty()) {
        RCLCPP_ERROR(get_logger(),
          "Arm '%s': no joints found in planning group — check SRDF", arm_name.c_str());
        continue;
      }

      arm_data->mgi = mgi;
      RCLCPP_INFO(get_logger(),
        "Arm '%s' ready: planning_frame='%s', ee_link='%s'",
        arm_name.c_str(),
        mgi->getPlanningFrame().c_str(),
        mgi->getEndEffectorLink().c_str());

      const moveit::core::JointModelGroup * jmg =
        mgi->getRobotModel()->getJointModelGroup(arm_data->config.planning_group);
      if (jmg) {
        std::unordered_map<std::string, double> vel_limits, accel_limits;
        for (const auto * jm : jmg->getActiveJointModels()) {
          const std::string & jname = jm->getName();
          const std::string prefix = "robot_description_planning.joint_limits." + jname + ".";
          if (this->has_parameter(prefix + "max_velocity")) {
            vel_limits[jname] =
              this->get_parameter(prefix + "max_velocity").as_double() * velocity_scaling_;
          }
          if (this->has_parameter(prefix + "has_acceleration_limits") &&
              this->get_parameter(prefix + "has_acceleration_limits").as_bool() &&
              this->has_parameter(prefix + "max_acceleration")) {
            accel_limits[jname] =
              this->get_parameter(prefix + "max_acceleration").as_double() * acceleration_scaling_;
          }
        }
        arm_data->vel_limits   = std::move(vel_limits);
        arm_data->accel_limits = std::move(accel_limits);
        RCLCPP_INFO(get_logger(),
          "Arm '%s': %zu active joints, cached vel_limits for %zu, accel_limits for %zu",
          arm_name.c_str(), jmg->getActiveJointModels().size(),
          arm_data->vel_limits.size(), arm_data->accel_limits.size());
      }

      // Publish mgi to other threads only once it and the limit caches are complete.
      arm_data->mgi_ready.store(true, std::memory_order_release);

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Failed to init MoveGroupInterface for '%s': %s", arm_name.c_str(), e.what());
    }
  }
}

// ── Joint state cache (for the topic-stop hold — never blocks the executor) ──

void MoveitArmController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // position can be shorter than name (velocity/effort-only publishers).
  const size_t n = std::min(msg->name.size(), msg->position.size());
  std::lock_guard<std::mutex> lk(joint_state_cache_mutex_);
  for (size_t i = 0; i < n; i++) {
    joint_state_cache_[msg->name[i]] = msg->position[i];
  }
}

// ── Enable / disable callback ───────────────────────────────────

void MoveitArmController::enable_callback(
  const std::string & arm_name,
  const std_msgs::msg::Bool::SharedPtr msg)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) return;
  auto & arm = *it->second;

  if (!arm.mgi_ready.load(std::memory_order_acquire)) {
    RCLCPP_WARN(get_logger(),
      "Arm '%s' MoveGroupInterface not ready yet — ignoring enable=true. "
      "Check that move_group is running and robot_description was fetched.",
      arm_name.c_str());
    return;
  }

  if (msg->data) {
    // Locked for the whole decision so it can't race the loop's own exit check.
    std::lock_guard<std::mutex> lk(arm.lifecycle_mutex);
    if (arm.enabled.load()) return;
    arm.enabled = true;

    if (!arm.thread_active.load()) {
      if (arm.thread.joinable()) arm.thread.join();  // finished thread — immediate
      arm.thread_active = true;
      arm.thread = std::thread([this, arm_name]() { tracking_loop(arm_name); });
      RCLCPP_INFO(get_logger(), "Arm tracking ENABLED for '%s'", arm_name.c_str());
    }
    // else: a wind-down loop will see enabled=true and restart tracking itself.
  } else {
    if (!arm.enabled.load()) return;
    arm.enabled = false;
    cancel_trajectory(arm);
    RCLCPP_INFO(get_logger(), "Arm tracking DISABLED for '%s'", arm_name.c_str());
  }
}

// ── Trajectory helpers ──────────────────────────────────────────

void MoveitArmController::send_trajectory(
  ArmData & arm,
  const trajectory_msgs::msg::JointTrajectory & jtraj)
{
  // ── Topic mode: stream the trajectory on the controller's command topic ──
  if (use_topic_) {
    trajectory_msgs::msg::JointTrajectory out = jtraj;
    out.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    arm.traj_pub->publish(out);
    return;
  }

  // ── Action mode (fallback): FollowJointTrajectory goal with preemption ──
  trajectory_msgs::msg::JointTrajectory action_jtraj = jtraj;
  action_jtraj.header.stamp =
    this->now() + rclcpp::Duration::from_nanoseconds(traj_lookahead_ms_ * 1'000'000LL);

  if (!arm.action_client->wait_for_action_server(std::chrono::milliseconds(0))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "follow_joint_trajectory action server not available");
    return;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = action_jtraj;

  auto send_opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  send_opts.result_callback =
    [this, &arm](const GoalHandleFJT::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(arm.goal_mutex);
        arm.goal_handle.reset();
      }
      arm.executing = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_DEBUG(get_logger(), "Trajectory succeeded");
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_DEBUG(get_logger(), "Trajectory canceled (preempted for replan)");
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Trajectory finished with code %d", static_cast<int>(result.code));
      }
    };

  arm.executing = true;
  auto gh_future = arm.action_client->async_send_goal(goal, send_opts);

  auto status = gh_future.wait_for(std::chrono::seconds(2));
  if (status != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Timed out waiting for goal acceptance");
    arm.executing = false;
    return;
  }

  auto gh = gh_future.get();
  if (!gh) {
    RCLCPP_WARN(get_logger(), "Goal was rejected by action server");
    arm.executing = false;
    return;
  }

  std::lock_guard<std::mutex> lock(arm.goal_mutex);
  arm.goal_handle = gh;
}

void MoveitArmController::cancel_trajectory(ArmData & arm)
{
  // The cancelled trajectory must never be used as the next cycle's streaming
  // seed (may be called from the enable_callback thread, hence the mutex).
  {
    std::lock_guard<std::mutex> lk(arm.last_sent_mutex);
    arm.last_sent_traj.reset();
  }

  // Topic mode: halt by commanding the controller to hold the current joint
  // positions. Uses the joint_states cache, not MGI's state monitor — this
  // may run on the enable_callback/executor thread and must never block it.
  if (use_topic_) {
    if (!arm.mgi_ready.load(std::memory_order_acquire)) return;
    std::vector<std::string> names = arm.mgi->getJoints();  // cached local call, no service
    std::vector<double> pos;
    pos.reserve(names.size());
    {
      std::lock_guard<std::mutex> lk(joint_state_cache_mutex_);
      for (const auto & name : names) {
        auto it = joint_state_cache_.find(name);
        if (it == joint_state_cache_.end()) {
          RCLCPP_WARN(get_logger(),
            "topic-stop: joint '%s' missing from joint_states cache — arm holds last trajectory",
            name.c_str());
          return;  // never publish empty/partial names
        }
        pos.push_back(it->second);
      }
    }
    if (names.empty()) {
      RCLCPP_WARN(get_logger(),
        "topic-stop: skipped hold (no joints) — arm holds last trajectory");
      return;  // never publish empty names
    }
    RCLCPP_INFO(get_logger(), "topic-stop: holding %zu joints at current position",
      names.size());

    trajectory_msgs::msg::JointTrajectory hold;
    hold.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    hold.joint_names = std::move(names);
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = std::move(pos);
    pt.time_from_start = rclcpp::Duration::from_seconds(0.1);
    hold.points.push_back(std::move(pt));
    arm.traj_pub->publish(hold);
    return;
  }

  std::shared_ptr<GoalHandleFJT> gh;
  {
    std::lock_guard<std::mutex> lock(arm.goal_mutex);
    gh = arm.goal_handle;
  }
  if (gh && arm.executing.load()) {
    arm.action_client->async_cancel_goal(gh);
  }
}

// ── Tracking loop (per-arm thread) ──────────────────────────────

void MoveitArmController::tracking_loop(const std::string & arm_name)
{
  auto & arm = *arms_[arm_name];
  auto & mgi = arm.mgi;
  const auto & cfg = arm.config;

  RCLCPP_INFO(get_logger(), "Tracking loop started for arm '%s'", arm_name.c_str());

  const std::string ee_link = mgi->getEndEffectorLink();
  if (ee_link.empty()) {
    RCLCPP_ERROR(get_logger(),
      "Arm '%s': no end-effector link configured (check SRDF) — tracking loop exiting",
      arm_name.c_str());
    return;
  }

  const moveit::core::JointModelGroup * jmg =
    mgi->getRobotModel()->getJointModelGroup(cfg.planning_group);
  if (!jmg) {
    RCLCPP_ERROR(get_logger(),
      "Arm '%s': planning group '%s' not found in robot model — tracking loop exiting",
      arm_name.c_str(), cfg.planning_group.c_str());
    return;
  }

  // Tripwire: getGlobalLinkTransform() is in the model root frame, which must
  // match cfg.base_frame — no re-transform is done below.
  {
    std::string planning_frame = mgi->getPlanningFrame();
    std::string base_frame     = cfg.base_frame;
    if (!planning_frame.empty() && planning_frame.front() == '/') {
      planning_frame = planning_frame.substr(1);
    }
    if (!base_frame.empty() && base_frame.front() == '/') {
      base_frame = base_frame.substr(1);
    }
    if (planning_frame != base_frame) {
      RCLCPP_WARN(get_logger(),
        "Arm '%s': planning frame '%s' != base_frame '%s' — EE pose math in "
        "tracking_loop assumes these match and does NOT re-transform",
        arm_name.c_str(), mgi->getPlanningFrame().c_str(), cfg.base_frame.c_str());
    }
  }

  const double lookahead_s = traj_lookahead_ms_ / 1000.0;

  // Outer loop: a re-enable during wind-down restarts tracking here instead of
  // leaving enabled=true with no loop running (see lifecycle_mutex below).
  for (;;) {

  rclcpp::Rate rate(update_rate_hz_);
  geometry_msgs::msg::Pose last_submitted_target;
  bool first_iter = true;
  arm.last_heartbeat_sec = this->now().seconds();

  // Invalidate any seed left over from a previous enable/disable cycle.
  {
    std::lock_guard<std::mutex> lk(arm.last_sent_mutex);
    arm.last_sent_traj.reset();
  }

  while (rclcpp::ok() && arm.enabled.load()) {
    // ── 1. Look up target TF ──────────────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        cfg.base_frame, cfg.target_frame,
        tf2::TimePointZero, tf2::Duration(0));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed for arm '%s': %s", arm_name.c_str(), e.what());
      rate.sleep();
      continue;
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x  = tf_stamped.transform.translation.x;
    target_pose.position.y  = tf_stamped.transform.translation.y;
    target_pose.position.z  = tf_stamped.transform.translation.z;
    target_pose.orientation = tf_stamped.transform.rotation;

    // ── 2. Get measured state, then build the SEED state ──────────────────
    // Seed from the in-flight commanded state (sampled at now()+lookahead), not
    // the lagging measured state, so hops chain forward smoothly.
    moveit::core::RobotStatePtr measured_state = mgi->getCurrentState();
    if (!measured_state) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Arm '%s': current state unavailable — skipping cycle", arm_name.c_str());
      rate.sleep();
      continue;
    }

    moveit::core::RobotStatePtr seed;
    {
      robot_trajectory::RobotTrajectoryPtr prev_traj;
      rclcpp::Time prev_time;
      {
        std::lock_guard<std::mutex> lk(arm.last_sent_mutex);
        prev_traj = arm.last_sent_traj;
        prev_time = arm.last_sent_time;
      }

      if (prev_traj) {
        double elapsed = (this->now() - prev_time).seconds() + lookahead_s;
        if (elapsed < prev_traj->getDuration()) {
          seed = std::make_shared<moveit::core::RobotState>(*measured_state);
          prev_traj->getStateAtDurationFromStart(elapsed, seed);

          // interpolate() blends positions only — blend velocity/acceleration
          // explicitly so the seed carries the in-flight commanded velocity.
          const std::size_t n = prev_traj->getWayPointCount();
          std::size_t after = 0;
          while (after < n &&
                 prev_traj->getWayPointDurationFromStart(after) < elapsed) {
            ++after;
          }
          if (after >= n) after = n - 1;
          const std::size_t before = (after > 0) ? after - 1 : 0;
          double blend = 0.0;
          if (after > before) {
            const double t_before = prev_traj->getWayPointDurationFromStart(before);
            const double t_after  = prev_traj->getWayPointDurationFromStart(after);
            if (t_after > t_before) {
              blend = (elapsed - t_before) / (t_after - t_before);
            }
          }
          std::vector<double> vel_before, vel_after, accel_before, accel_after;
          prev_traj->getWayPoint(before).copyJointGroupVelocities(jmg, vel_before);
          prev_traj->getWayPoint(after).copyJointGroupVelocities(jmg, vel_after);
          prev_traj->getWayPoint(before).copyJointGroupAccelerations(jmg, accel_before);
          prev_traj->getWayPoint(after).copyJointGroupAccelerations(jmg, accel_after);
          for (std::size_t k = 0; k < vel_before.size() && k < vel_after.size(); ++k) {
            vel_before[k] += (vel_after[k] - vel_before[k]) * blend;
          }
          for (std::size_t k = 0; k < accel_before.size() && k < accel_after.size(); ++k) {
            accel_before[k] += (accel_after[k] - accel_before[k]) * blend;
          }
          seed->setJointGroupVelocities(jmg, vel_before);
          seed->setJointGroupAccelerations(jmg, accel_before);
        }
      }
      if (!seed) {
        seed = std::make_shared<moveit::core::RobotState>(*measured_state);
        seed->zeroVelocities();
      }
    }

    const Eigen::Isometry3d & seed_ee_tf = seed->getGlobalLinkTransform(ee_link);
    geometry_msgs::msg::Pose current_pose = tf2::toMsg(seed_ee_tf);

    // ── 3. Distance to target ── position AND orientation must both be within
    // threshold, else a pure reorientation would read as already-there.
    double dist = pose_distance(current_pose, target_pose);
    double ang  = pose_angle(current_pose, target_pose);
    if (dist < arrival_threshold_m_ && ang < arrival_threshold_rad_) {
      rate.sleep();
      continue;
    }

    // ── 4. Execution / preemption logic ───────────────────────────────────
    if (arm.executing.load()) {
      double target_moved   = pose_distance(target_pose, last_submitted_target);
      double target_rotated = pose_angle(target_pose, last_submitted_target);
      if (target_moved < preempt_threshold_m_ && target_rotated < preempt_threshold_rad_) {
        rate.sleep();
        continue;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "Arm '%s': target moved %.3f m / %.1f deg — preempting trajectory",
        arm_name.c_str(), target_moved, target_rotated * 180.0 / M_PI);
      cancel_trajectory(arm);
      std::this_thread::sleep_for(std::chrono::milliseconds(preempt_settle_ms_));
    }

    // ── 5. Replan decision (when not executing) ───────────────────────────
    if (!first_iter && !arm.executing.load()) {
      auto now_sec = this->now().seconds();
      bool heartbeat = (now_sec - arm.last_heartbeat_sec >= heartbeat_period_sec_);
      double target_moved   = pose_distance(target_pose, last_submitted_target);
      double target_rotated = pose_angle(target_pose, last_submitted_target);
      if (!heartbeat &&
          target_moved < replan_threshold_m_ &&
          target_rotated < replan_threshold_rad_) {
        rate.sleep();
        continue;
      }
      if (heartbeat) arm.last_heartbeat_sec = now_sec;
    }

    // ── 6. Clamp step to max_cartesian_step_m_ ────────────────────────────
    geometry_msgs::msg::Pose step_target = target_pose;
    if (dist > max_cartesian_step_m_) {
      double scale = max_cartesian_step_m_ / dist;
      step_target.position.x = current_pose.position.x +
        (target_pose.position.x - current_pose.position.x) * scale;
      step_target.position.y = current_pose.position.y +
        (target_pose.position.y - current_pose.position.y) * scale;
      step_target.position.z = current_pose.position.z +
        (target_pose.position.z - current_pose.position.z) * scale;
      tf2::Quaternion q_c, q_t;
      tf2::fromMsg(current_pose.orientation, q_c);
      tf2::fromMsg(target_pose.orientation, q_t);
      step_target.orientation = tf2::toMsg(q_c.slerp(q_t, scale));
    }

    // ── 7. Compute Cartesian path ─────────────────────────────────────────
    std::unique_lock<std::mutex> plan_lock(planning_mutex_);

    mgi->setStartState(*seed);
    std::vector<geometry_msgs::msg::Pose> waypoints = {step_target};
    moveit_msgs::msg::RobotTrajectory traj_msg;

    auto t0 = std::chrono::steady_clock::now();
    double fraction = mgi->computeCartesianPath(
      waypoints, eef_step_m_, traj_msg, avoid_collisions_);
    auto plan_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();

    if (first_iter) {
      RCLCPP_INFO(get_logger(),
        "Arm '%s': first plan — dist=%.3f m, step=%.3f m, fraction=%.2f, plan=%ldms",
        arm_name.c_str(), dist,
        std::min(dist, max_cartesian_step_m_),
        fraction, plan_ms);
    }

    const double period_ms = 1000.0 / update_rate_hz_;
    if (plan_ms > static_cast<long>(period_ms)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Arm '%s': slow plan %ldms (target %.1fms), dist=%.3f m, fraction=%.2f",
        arm_name.c_str(), plan_ms, period_ms, dist, fraction);
    }

    if (fraction < min_cartesian_fraction_) {
      // ── 7b. Cartesian failed — fall back to OMPL ─────────────────────────
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Arm '%s': Cartesian fraction %.2f < %.2f — falling back to OMPL "
        "(step_target xyz=[%.3f, %.3f, %.3f])",
        arm_name.c_str(), fraction, min_cartesian_fraction_,
        step_target.position.x, step_target.position.y, step_target.position.z);

      mgi->setPlanningTime(ompl_planning_timeout_s_);
      mgi->setPoseTarget(step_target);
      moveit::planning_interface::MoveGroupInterface::Plan ompl_plan;
      auto ompl_result = mgi->plan(ompl_plan);

      if (ompl_result != moveit::core::MoveItErrorCode::SUCCESS) {
        plan_lock.unlock();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "OMPL plan failed for arm '%s' (code %d) — target xyz=[%.3f, %.3f, %.3f]",
          arm_name.c_str(), ompl_result.val,
          step_target.position.x, step_target.position.y, step_target.position.z);
        rate.sleep();
        continue;
      }

      trajectory_msgs::msg::JointTrajectory jtraj_ompl =
        ompl_plan.trajectory.joint_trajectory;
      plan_lock.unlock();
      send_trajectory(arm, jtraj_ompl);

      // OMPL sweeps aren't re-timed with the seed velocity, so don't use one as
      // the next streaming seed — clear instead.
      {
        std::lock_guard<std::mutex> lk(arm.last_sent_mutex);
        arm.last_sent_traj.reset();
      }

      last_submitted_target = step_target;
      first_iter = false;
      rate.sleep();
      continue;
    }

    // ── 8. Time-parameterise ──────────────────────────────────────────────
    auto robot_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
      mgi->getRobotModel(), cfg.planning_group);
    robot_traj->setRobotTrajectoryMsg(*seed, traj_msg);

    constexpr double kMinWaypointSeparation = 0.01;  // rad (L1 over the group)
    {
      auto deduped = std::make_shared<robot_trajectory::RobotTrajectory>(
        mgi->getRobotModel(), cfg.planning_group);
      for (size_t i = 0; i < robot_traj->getWayPointCount(); ++i) {
        const moveit::core::RobotState & wp = robot_traj->getWayPoint(i);
        if (deduped->getWayPointCount() == 0 ||
            deduped->getLastWayPoint().distance(wp, jmg) > kMinWaypointSeparation) {
          deduped->addSuffixWayPoint(wp, 0.0);
        }
      }
      if (deduped->getWayPointCount() < 2) {
        plan_lock.unlock();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Arm '%s': degenerate trajectory (%zu distinct waypoints) — skipping",
          arm_name.c_str(), deduped->getWayPointCount());
        rate.sleep();
        continue;
      }
      robot_traj = deduped;
    }

    // Use the scaling-factor overload velocity/acceleration limit vectors
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool totg_ok =
      totg.computeTimeStamps(*robot_traj, velocity_scaling_, acceleration_scaling_);

    if (!totg_ok) {
      plan_lock.unlock();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "TOTG failed for arm '%s'", arm_name.c_str());
      rate.sleep();
      continue;
    }

    // ── 8b. Velocity-continuous retiming ── TOTG starts from rest; seed waypoint 0
    // with the in-flight velocity and re-smooth with Ruckig to kill the stop-start sawtooth.
    {
      moveit::core::RobotState & wp0 = *robot_traj->getFirstWayPointPtr();
      std::vector<double> seed_vel, seed_accel;
      seed->copyJointGroupVelocities(jmg, seed_vel);
      seed->copyJointGroupAccelerations(jmg, seed_accel);
      wp0.setJointGroupVelocities(jmg, seed_vel);
      wp0.setJointGroupAccelerations(jmg, seed_accel);
    }

    bool smoothed = trajectory_processing::RuckigSmoothing::applySmoothing(
      *robot_traj, velocity_scaling_, acceleration_scaling_);
    if (!smoothed) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Arm '%s': Ruckig smoothing failed — falling back to TOTG timing "
        "(trajectory will restart at zero velocity)", arm_name.c_str());
    }

    robot_traj->getRobotTrajectoryMsg(traj_msg);
    plan_lock.unlock();

    // ── 9. Send (topic publish or action goal, per publish_mode) ──────────
    trajectory_msgs::msg::JointTrajectory jtraj = traj_msg.joint_trajectory;
    send_trajectory(arm, jtraj);

    {
      std::lock_guard<std::mutex> lk(arm.last_sent_mutex);
      arm.last_sent_traj = robot_traj;
      arm.last_sent_time = this->now();
    }

    last_submitted_target = step_target;
    first_iter = false;

    rate.sleep();
  }

  cancel_trajectory(arm);
  mgi->stop();

  {
    std::lock_guard<std::mutex> lk(arm.lifecycle_mutex);
    if (!(rclcpp::ok() && arm.enabled.load())) { arm.thread_active = false; break; }
  }
  // re-enabled during wind-down: outer loop restarts tracking in this same thread
  RCLCPP_INFO(get_logger(), "Arm '%s': re-enabled during wind-down — restarting", arm_name.c_str());
  }  // for (;;)

  RCLCPP_INFO(get_logger(), "Tracking loop ended for arm '%s'", arm_name.c_str());
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::MoveitArmController)
