#include "sobits_teleop/servo_target_bridge.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>

namespace sobits_teleop
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

ServoTargetBridge::ServoTargetBridge(const rclcpp::NodeOptions & options)
: Node(
    "servo_target_bridge",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!this->has_parameter("servo_bridge.pose_rate_hz"))
    this->declare_parameter("servo_bridge.pose_rate_hz", 100.0);
  pose_rate_hz_ = this->get_parameter("servo_bridge.pose_rate_hz").as_double();

  if (!this->has_parameter("servo_bridge.arms"))
    this->declare_parameter("servo_bridge.arms",
      std::vector<std::string>{"arm_right", "arm_left"});

  auto arm_names = this->get_parameter("servo_bridge.arms").as_string_array();

  for (const auto & arm_name : arm_names) {
    auto tf_key   = "servo_bridge." + arm_name + ".target_frame";
    auto bf_key   = "servo_bridge." + arm_name + ".base_frame";
    auto sn_key   = "servo_bridge." + arm_name + ".servo_node";
    auto en_key   = "servo_bridge." + arm_name + ".enable_topic";
    auto ro_key   = "servo_bridge." + arm_name + ".reach_origin_frame";
    auto mr_key   = "servo_bridge." + arm_name + ".max_reach";

    if (!this->has_parameter(tf_key))
      this->declare_parameter(tf_key, arm_name + "_target_link");
    if (!this->has_parameter(bf_key))
      this->declare_parameter(bf_key, "base_footprint");
    if (!this->has_parameter(sn_key))
      this->declare_parameter(sn_key, std::string("servo_") + arm_name);
    if (!this->has_parameter(en_key))
      this->declare_parameter(en_key, arm_name + "/moveit_track_enabled");
    // reach_origin_frame/max_reach default disabled; the launch YAML supplies them.
    if (!this->has_parameter(ro_key))
      this->declare_parameter(ro_key, std::string(""));
    if (!this->has_parameter(mr_key))
      this->declare_parameter(mr_key, 0.0);

    ServoBridgeArmConfig cfg;
    cfg.target_frame   = this->get_parameter(tf_key).as_string();
    cfg.base_frame     = this->get_parameter(bf_key).as_string();
    cfg.servo_node     = this->get_parameter(sn_key).as_string();
    cfg.enable_topic   = this->get_parameter(en_key).as_string();
    cfg.reach_origin_frame = this->get_parameter(ro_key).as_string();
    cfg.max_reach      = this->get_parameter(mr_key).as_double();

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

    // Enable subscription: reliable + transient_local so a late-starting Servo
    // bridge still receives the current enable state even if it started after
    // sobits_teleop published it (see D6 — the sobits_teleop publisher side is
    // switched to reliable/transient_local/depth1 to match).
    rclcpp::QoS enable_qos(1);
    enable_qos.reliable();
    enable_qos.transient_local();

    arm_data->enable_sub = this->create_subscription<std_msgs::msg::Bool>(
      cfg.enable_topic, enable_qos,
      [this, arm_name](const std_msgs::msg::Bool::SharedPtr msg) {
        enable_callback(arm_name, msg);
      });

    arms_[arm_name] = std::move(arm_data);

    RCLCPP_INFO(get_logger(),
      "Arm '%s': servo_node='%s', target_frame='%s', base_frame='%s', enable_topic='%s', "
      "reach_origin_frame='%s', max_reach=%.3f",
      arm_name.c_str(), cfg.servo_node.c_str(), cfg.target_frame.c_str(),
      cfg.base_frame.c_str(), cfg.enable_topic.c_str(),
      cfg.reach_origin_frame.c_str(), cfg.max_reach);

    // Startup sequence — switch_command_type(POSE) once per arm and
    // pause_servo(true) so arms don't move until first enable. Both are async
    // and retried on a slow (2 s) timer until each succeeds independently
    // (servo_node's services are typically not up yet at bridge construction
    // time, since the launcher starts them concurrently).
    arms_[arm_name]->startup_retry_timer = this->create_wall_timer(
      std::chrono::seconds(2),
      [this, arm_name]() { try_startup_sequence(arm_name); });
    // Fire once immediately too, in case services are already up.
    try_startup_sequence(arm_name);
  }

  // One shared timer drives all arms' pose publishing at pose_rate_hz_.
  auto period = std::chrono::duration<double>(1.0 / pose_rate_hz_);
  pose_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { pose_timer_callback(); });

  RCLCPP_INFO(get_logger(),
    "ServoTargetBridge: %zu arm(s), pose_rate=%.1f Hz",
    arms_.size(), pose_rate_hz_);
}

// ---------------------------------------------------------------------------
// Startup sequence: switch_command_type(POSE) + pause_servo(true), each async
// and retried independently on a slow timer until both have succeeded.
// ---------------------------------------------------------------------------

void ServoTargetBridge::try_startup_sequence(const std::string & arm_name)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) return;
  auto & arm = *it->second;

  if (arm.command_type_set.load() && arm.initial_pause_set.load()) {
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
          if (it2 == arms_.end()) return;
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
          if (it2 == arms_.end()) return;
          auto & arm2 = *it2->second;
          auto resp = future.get();
          if (resp->success) {
            arm2.initial_pause_set = true;
            RCLCPP_INFO(get_logger(),
              "Arm '%s': initial pause_servo(true) succeeded on '%s'",
              arm_name.c_str(), arm2.config.servo_node.c_str());
            // Race guard: an enable arrived while this startup pause was in
            // flight — it just froze an arm the operator believes is enabled.
            // Undo immediately with an async unpause.
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

// ---------------------------------------------------------------------------
// Enable / disable callback (async only — never blocks the executor)
// ---------------------------------------------------------------------------

void ServoTargetBridge::enable_callback(
  const std::string & arm_name,
  const std_msgs::msg::Bool::SharedPtr msg)
{
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) return;
  auto & arm = *it->second;

  if (msg->data) {
    if (arm.enabled.load()) return;

    // Don't wait for service responses before publishing — Servo tolerates
    // early pose commands while paused. Ensure POSE mode is (re-)requested in
    // case the servo_node restarted after our one-time startup call, then
    // unpause.
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

    if (arm.pause_servo_client->service_is_ready()) {
      auto req = std::make_shared<SetBool::Request>();
      req->data = false;
      arm.pause_servo_client->async_send_request(
        req,
        [this, arm_name](rclcpp::Client<SetBool>::SharedFuture future) {
          auto resp = future.get();
          RCLCPP_INFO(get_logger(), "Arm '%s': pause_servo(false) on enable -> %s",
            arm_name.c_str(), resp->success ? "ok" : "FAILED");
        });
    }

    arm.enabled = true;
    RCLCPP_INFO(get_logger(), "Servo tracking ENABLED for '%s'", arm_name.c_str());
  } else {
    if (!arm.enabled.load()) return;
    arm.enabled = false;

    if (arm.pause_servo_client->service_is_ready()) {
      auto req = std::make_shared<SetBool::Request>();
      req->data = true;
      arm.pause_servo_client->async_send_request(
        req,
        [this, arm_name](rclcpp::Client<SetBool>::SharedFuture future) {
          auto resp = future.get();
          RCLCPP_INFO(get_logger(), "Arm '%s': pause_servo(true) on disable -> %s",
            arm_name.c_str(), resp->success ? "ok" : "FAILED");
        });
    }

    RCLCPP_INFO(get_logger(), "Servo tracking DISABLED for '%s'", arm_name.c_str());
  }
}

// ---------------------------------------------------------------------------
// Shared pose-publish timer
// ---------------------------------------------------------------------------

void ServoTargetBridge::pose_timer_callback()
{
  for (auto & [arm_name, arm_ptr] : arms_) {
    auto & arm = *arm_ptr;
    if (!arm.enabled.load()) continue;

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

    // Reach clamp: pull an out-of-reach target back onto the max_reach sphere
    // around the shoulder. Prevents the arm from chasing an unreachable hand
    // target into full extension (elbow singularity -> latched servo e-stop).
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

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = arm.config.base_frame;
    pose_msg.header.stamp    = this->now();
    pose_msg.pose.position.x = base_to_target.getOrigin().x();
    pose_msg.pose.position.y = base_to_target.getOrigin().y();
    pose_msg.pose.position.z = base_to_target.getOrigin().z();
    pose_msg.pose.orientation = tf2::toMsg(base_to_target.getRotation());

    arm.pose_pub->publish(pose_msg);
  }
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::ServoTargetBridge)
