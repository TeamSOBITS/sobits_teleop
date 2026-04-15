// arm_scale_calibrator — measures human arm reach from Quest controller TFs
// and computes the correct scale value for quest.yaml.
//
// Usage:
//   ros2 run sobits_teleop arm_scale_calibrator [--ros-args -p joy_topic:=/sobit_home/joy]
//
// Control via the Quest right grip button (button index 7):
//   - First press+release  → start recording (begin sweep)
//   - Second press+release → stop recording, print result
//
// Robot arm reach (shoulder to EE): 1.2926 m  (from sobit_home URDF)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <mutex>

// Full kinematic chain shoulder→EE from sobit_home URDF (metres)
static constexpr double ROBOT_ARM_REACH_M = 1.2926;

static const std::string RIGHT_FRAME  = "right_controller_odom";
static const std::string LEFT_FRAME   = "left_controller_odom";
static const std::string PARENT_FRAME = "odom";

// Right grip axis index in the Joy message (quest.yaml: arm_mode: 7 — it's an axis, not a button)
static constexpr int GRIP_AXIS = 7;

struct Sample { double x, y, z; };

static double dist3(const Sample & a, const Sample & b)
{
  return std::sqrt(std::pow(a.x-b.x,2) + std::pow(a.y-b.y,2) + std::pow(a.z-b.z,2));
}

enum class State { WAITING_FOR_START, RECORDING, DONE };

class ArmScaleCalibrator : public rclcpp::Node
{
public:
  ArmScaleCalibrator()
  : Node("arm_scale_calibrator"),
    tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
    tf_listener_(tf_buffer_),
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
      ROBOT_ARM_REACH_M);
  }

  bool is_done() const { return state_ == State::DONE; }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (GRIP_AXIS >= static_cast<int>(msg->axes.size())) return;

    bool grip = (msg->axes[GRIP_AXIS] > 0.5f);

    // Ignore all edges until we have seen at least one message with grip released.
    // This prevents a grip that is already held at startup from immediately
    // triggering step 1 and then step 2 in rapid succession.
    if (!seen_grip_released_) {
      if (!grip) seen_grip_released_ = true;
      prev_grip_ = grip;
      return;
    }

    bool falling_edge = !grip && prev_grip_;  // button just released
    prev_grip_ = grip;

    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == State::WAITING_FOR_START && falling_edge) {
      // Capture start positions
      try {
        auto ts_r = tf_buffer_.lookupTransform(PARENT_FRAME, RIGHT_FRAME, tf2::TimePointZero);
        auto ts_l = tf_buffer_.lookupTransform(PARENT_FRAME, LEFT_FRAME,  tf2::TimePointZero);
        right_start_ = {ts_r.transform.translation.x,
                        ts_r.transform.translation.y,
                        ts_r.transform.translation.z};
        left_start_  = {ts_l.transform.translation.x,
                        ts_l.transform.translation.y,
                        ts_l.transform.translation.z};
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
        left_start_.x,  left_start_.y,  left_start_.z);
    }
    else if (state_ == State::RECORDING && falling_edge) {
      // Require at least 2 seconds of recording to avoid accidental double-tap
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

  void sample_cb()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::RECORDING) return;

    try {
      auto ts_r = tf_buffer_.lookupTransform(PARENT_FRAME, RIGHT_FRAME, tf2::TimePointZero);
      auto ts_l = tf_buffer_.lookupTransform(PARENT_FRAME, LEFT_FRAME,  tf2::TimePointZero);
      right_samples_.push_back({ts_r.transform.translation.x,
                                ts_r.transform.translation.y,
                                ts_r.transform.translation.z});
      left_samples_.push_back({ts_l.transform.translation.x,
                               ts_l.transform.translation.y,
                               ts_l.transform.translation.z});
    } catch (const tf2::TransformException &) {}
  }

  void compute_and_print()
  {
    if (right_samples_.empty() || left_samples_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No samples collected — did the sweep happen?");
      return;
    }

    double right_max = 0.0, left_max = 0.0;
    for (const auto & s : right_samples_) right_max = std::max(right_max, dist3(s, right_start_));
    for (const auto & s : left_samples_)  left_max  = std::max(left_max,  dist3(s, left_start_));

    double human_reach = std::max(right_max, left_max);

    if (human_reach < 0.05) {
      RCLCPP_ERROR(this->get_logger(),
        "Displacement too small (%.3f m) — re-run and sweep wider.", human_reach);
      return;
    }

    double scale = ROBOT_ARM_REACH_M / human_reach;

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
      ROBOT_ARM_REACH_M,
      ROBOT_ARM_REACH_M, human_reach, scale,
      scale);
  }

  tf2_ros::Buffer                                    tf_buffer_;
  tf2_ros::TransformListener                         tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr                       sample_timer_;

  std::mutex      mutex_;
  State           state_;
  bool            prev_grip_;
  bool            seen_grip_released_;
  rclcpp::Time    recording_start_;
  Sample          right_start_{}, left_start_{};
  std::vector<Sample> right_samples_, left_samples_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmScaleCalibrator>();
  while (rclcpp::ok() && !node->is_done()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // spin once more to flush the final log
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
