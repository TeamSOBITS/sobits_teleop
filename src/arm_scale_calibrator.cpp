// arm_scale_calibrator — measures human arm reach from Quest TFs and prints the quest.yaml scale.
// Right grip press+release starts recording; a second press+release stops and prints.

#include "sobits_teleop/arm_scale_calibrator.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace sobits_teleop
{

static double dist3(const Sample & a, const Sample & b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

ArmScaleCalibrator::ArmScaleCalibrator(const rclcpp::NodeOptions & options)
: Node("arm_scale_calibrator", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(tf_buffer_),
  robot_arm_reach_m_(this->declare_parameter<double>("robot_arm_reach_m", 1.2926)),
  right_frame_(this->declare_parameter<std::string>("right_frame", "right_controller_odom")),
  left_frame_(this->declare_parameter<std::string>("left_frame", "left_controller_odom")),
  parent_frame_(this->declare_parameter<std::string>("parent_frame", "base_footprint")),
  grip_axis_(this->declare_parameter<int>("grip_axis", 7)),
  state_(State::WAITING_FOR_START),
  prev_grip_(false),
  seen_grip_released_(false),
  recording_start_(this->now())
{
  std::string joy_topic = this->declare_parameter<std::string>(
    "joy_topic", "/sobit_home/joy");

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 10,
    std::bind(&ArmScaleCalibrator::joy_cb, this, std::placeholders::_1));

  sample_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&ArmScaleCalibrator::sample_cb, this));

  RCLCPP_INFO(this->get_logger(),
    "\n\n=== Arm Scale Calibrator ===\n"
    "Robot arm reach (shoulder to EE): %.4f m\n\n"
    "STEP 1: Hold BOTH controllers.\n"
    "        Extend BOTH arms straight FORWARD (pointing at the robot).\n"
    "        Press and release RIGHT GRIP to start recording.",
    robot_arm_reach_m_);
}

void ArmScaleCalibrator::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Guard against a negative grip_axis config typo, not just out-of-range.
  if (grip_axis_ < 0 || grip_axis_ >= static_cast<int>(msg->axes.size())) {return;}

  bool grip = (msg->axes[grip_axis_] > 0.5f);

  // Ignore edges until the grip has been seen released, so a grip held at
  // startup doesn't fire both steps at once.
  if (!seen_grip_released_) {
    if (!grip) {seen_grip_released_ = true;}
    prev_grip_ = grip;
    return;
  }

  bool falling_edge = !grip && prev_grip_;
  prev_grip_ = grip;

  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ == State::WAITING_FOR_START && falling_edge) {
    try {
      if (!right_frame_.empty()) {
        auto ts_r = tf_buffer_.lookupTransform(parent_frame_, right_frame_, tf2::TimePointZero);
        right_start_ = {ts_r.transform.translation.x,
          ts_r.transform.translation.y,
          ts_r.transform.translation.z};
      }
      if (!left_frame_.empty()) {
        auto ts_l = tf_buffer_.lookupTransform(parent_frame_, left_frame_, tf2::TimePointZero);
        left_start_ = {ts_l.transform.translation.x,
          ts_l.transform.translation.y,
          ts_l.transform.translation.z};
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(), "TF not ready: %s — try again", e.what());
      return;
    }

    right_samples_.clear();
    left_samples_.clear();
    recording_start_ = this->now();
    state_ = State::RECORDING;

    RCLCPP_INFO(this->get_logger(),
      "\nStart positions captured:\n"
      "  Right: (%.3f, %.3f, %.3f)\n"
      "  Left:  (%.3f, %.3f, %.3f)\n\n"
      "STEP 2: NOW slowly sweep both arms OUT to your sides,\n"
      "        elbows straight, until you reach a full T-pose.\n"
      "        Press and release RIGHT GRIP when arms are fully sideways.",
      right_start_.x, right_start_.y, right_start_.z,
      left_start_.x, left_start_.y, left_start_.z);
  } else if (state_ == State::RECORDING && falling_edge) {
    auto elapsed = this->now() - recording_start_;
    if (elapsed.seconds() < 2.0) {
      RCLCPP_WARN(this->get_logger(),
        "Too fast (%.1f s) — keep sweeping and grip again when arms are fully sideways.",
        elapsed.seconds());
      return;
    }
    state_ = State::DONE;
    compute_and_print();
  }
}

void ArmScaleCalibrator::sample_cb()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::RECORDING) {return;}

  try {
    if (!right_frame_.empty()) {
      auto ts_r = tf_buffer_.lookupTransform(parent_frame_, right_frame_, tf2::TimePointZero);
      right_samples_.push_back({ts_r.transform.translation.x,
          ts_r.transform.translation.y,
          ts_r.transform.translation.z});
    }
    if (!left_frame_.empty()) {
      auto ts_l = tf_buffer_.lookupTransform(parent_frame_, left_frame_, tf2::TimePointZero);
      left_samples_.push_back({ts_l.transform.translation.x,
          ts_l.transform.translation.y,
          ts_l.transform.translation.z});
    }
  } catch (const tf2::TransformException &) {
  }
}

void ArmScaleCalibrator::compute_and_print()
{
  const bool has_right = !right_frame_.empty() && !right_samples_.empty();
  const bool has_left = !left_frame_.empty() && !left_samples_.empty();

  if (!has_right && !has_left) {
    RCLCPP_ERROR(this->get_logger(), "No samples collected — did the sweep happen?");
    return;
  }

  double right_max = 0.0, left_max = 0.0;
  if (has_right) {
    for (const auto & s : right_samples_) {
      right_max = std::max(right_max, dist3(s, right_start_));
    }
  }
  if (has_left) {
    for (const auto & s : left_samples_) {
      left_max = std::max(left_max, dist3(s, left_start_));
    }
  }

  double human_reach = std::max(right_max, left_max);

  if (human_reach < 0.05) {
    RCLCPP_ERROR(this->get_logger(),
      "Displacement too small (%.3f m) — re-run and sweep wider.", human_reach);
    return;
  }

  double scale = robot_arm_reach_m_ / human_reach;

  RCLCPP_INFO(this->get_logger(),
    "\n\n=== RESULTS ===\n"
    "  Samples collected      : %zu\n"
    "  Right max displacement : %.4f m\n"
    "  Left  max displacement : %.4f m\n"
    "  Human arm reach used   : %.4f m\n"
    "  Robot arm reach        : %.4f m\n\n"
    "  Recommended scale = %.4f / %.4f = %.4f\n\n"
    "Update config/sobit_home/quest.yaml:\n"
    "    scale: %.4f   # (both right and left)",
    right_samples_.size(),
    right_max, left_max,
    human_reach,
    robot_arm_reach_m_,
    robot_arm_reach_m_, human_reach, scale,
    scale);

  rclcpp::shutdown();
}

}  // namespace sobits_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_teleop::ArmScaleCalibrator)
