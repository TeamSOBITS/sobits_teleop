#ifndef SOBITS_TELEOP__ARM_SCALE_CALIBRATOR_HPP_
#define SOBITS_TELEOP__ARM_SCALE_CALIBRATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <string>
#include <vector>

namespace sobits_teleop
{

struct Sample { double x, y, z; };

enum class State { WAITING_FOR_START, RECORDING, DONE };

class ArmScaleCalibrator : public rclcpp::Node
{
public:
  explicit ArmScaleCalibrator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void sample_cb();
  void compute_and_print();

  tf2_ros::Buffer                                      tf_buffer_;
  tf2_ros::TransformListener                           tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr                         sample_timer_;

  double      robot_arm_reach_m_;
  std::string right_frame_;
  std::string left_frame_;
  std::string parent_frame_;
  int         grip_axis_;

  std::mutex   mutex_;
  State        state_;
  bool         prev_grip_;
  bool         seen_grip_released_;
  rclcpp::Time recording_start_;
  Sample       right_start_{}, left_start_{};
  std::vector<Sample> right_samples_, left_samples_;
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__ARM_SCALE_CALIBRATOR_HPP_
