#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class ServoTwistTeleop final : public rclcpp::Node
{
public:
  ServoTwistTeleop()
  : Node("servo_twist_teleop")
  {
    mode_button_ = declare_parameter<int>("mode_button", 2);
    arm_button_ = declare_parameter<int>("arm_button", 3);
    axes_ = declare_parameter<std::vector<int64_t>>("axes", {0, 1, 3});
    axis_signs_ = declare_parameter<std::vector<double>>("axis_signs", {1.0, 1.0, 1.0});
    linear_speed_ = declare_parameter<double>("linear_speed", 0.06);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.25);
    deadzone_ = declare_parameter<double>("deadzone", 0.12);
    timeout_s_ = declare_parameter<double>("timeout_s", 0.25);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_footprint");
    arms_ = declare_parameter<std::vector<std::string>>("arms", {"arm_right", "arm_left"});

    if (axes_.size() != 3 || axis_signs_.size() != 3 || arms_.empty()) {
      throw std::runtime_error("axes and axis_signs must contain 3 entries; arms cannot be empty");
    }
    if (mode_button_ < 0 || arm_button_ < 0 ||
      linear_speed_ < 0.0 || angular_speed_ < 0.0 || deadzone_ < 0.0 || timeout_s_ <= 0.0)
    {
      throw std::runtime_error("Invalid teleop parameter: button indices must be nonnegative, speeds/deadzone nonnegative, timeout positive");
    }

    for (const auto & arm : arms_) {
      publishers_[arm] = create_publisher<geometry_msgs::msg::TwistStamped>(
        "servo_" + arm + "/delta_twist_cmds", 10);
      clients_[arm] = create_client<moveit_msgs::srv::ServoCommandType>(
        "servo_" + arm + "/switch_command_type");
      ready_[arm] = false;
      request_pending_[arm] = false;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&ServoTwistTeleop::on_joy, this, std::placeholders::_1));
    timer_ = create_wall_timer(20ms, std::bind(&ServoTwistTeleop::tick, this));
    last_joy_time_ = std::chrono::steady_clock::now();
  }

private:
  static bool pressed(const std::vector<int32_t> & buttons, int index)
  {
    return index >= 0 && static_cast<size_t>(index) < buttons.size() && buttons[index] != 0;
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto & buttons = msg->buttons;
    const bool mode_edge = pressed(buttons, mode_button_) && !pressed(previous_buttons_, mode_button_);
    const bool arm_edge = pressed(buttons, arm_button_) && !pressed(previous_buttons_, arm_button_);

    if (mode_edge) {
      rotation_mode_ = !rotation_mode_;
      RCLCPP_INFO(get_logger(), "%s mode", rotation_mode_ ? "Rotation" : "Translation");
    }
    if (arm_edge) {
      selected_arm_ = (selected_arm_ + 1) % arms_.size();
      RCLCPP_INFO(get_logger(), "Selected arm: %s", arms_[selected_arm_].c_str());
    }
    
    previous_buttons_ = buttons;
    latest_joy_ = *msg;
    have_joy_ = true;
    last_joy_time_ = std::chrono::steady_clock::now();
  }

  void request_twist_mode(const std::string & arm)
  {
    auto client = clients_.at(arm);
    if (request_pending_.at(arm) || !client->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
    request_pending_[arm] = true;
    client->async_send_request(request,
      [this, arm](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
        request_pending_[arm] = false;
        try {
          const auto response = future.get();
          ready_[arm] = response->success;
          if (ready_[arm]) {
            RCLCPP_INFO(get_logger(), "Servo for %s switched to TWIST mode", arm.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "Servo for %s rejected TWIST mode", arm.c_str());
          }
        } catch (const std::exception & e) {
          ready_[arm] = false;
          RCLCPP_WARN(get_logger(), "Servo mode switch failed for %s: %s", arm.c_str(), e.what());
        }
      });
  }

  geometry_msgs::msg::TwistStamped zero_command() const
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    return msg;
  }

  void tick()
  {
    const auto & arm = arms_[selected_arm_];
    if (!ready_[arm]) {
      request_twist_mode(arm);
      // Keep sending zero while waiting for the Servo mode handshake.
      publishers_[arm]->publish(zero_command());
      return;
    }

    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_joy_time_).count();
    const bool active = have_joy_ && age <= timeout_s_;

    std::vector<double> values(3, 0.0);
    if (active) {
      for (size_t i = 0; i < 3; ++i) {
        const int64_t axis = axes_[i];
        if (axis >= 0 && static_cast<size_t>(axis) < latest_joy_.axes.size()) {
          const double raw = static_cast<double>(latest_joy_.axes[axis]) * axis_signs_[i];
          if (std::isfinite(raw) && std::abs(raw) >= deadzone_) {
            values[i] = std::clamp(raw, -1.0, 1.0);
          }
        }
      }
    }

    auto msg = zero_command();
    const double speed = rotation_mode_ ? angular_speed_ : linear_speed_;
    auto & target = rotation_mode_ ? msg.twist.angular : msg.twist.linear;
    target.x = values[0] * speed;
    target.y = values[1] * speed;
    target.z = values[2] * speed;
    publishers_[arm]->publish(msg);

    // Stop all non-selected arms to avoid stale commands after switching arms.
    for (const auto & other : arms_) {
      if (other != arm && ready_[other]) {
        publishers_[other]->publish(zero_command());
      }
    }
  }

  int mode_button_{2};
  int arm_button_{3};
  std::vector<int64_t> axes_;
  std::vector<double> axis_signs_;
  double linear_speed_{0.06};
  double angular_speed_{0.25};
  double deadzone_{0.12};
  double timeout_s_{0.25};
  std::string frame_id_{"base_footprint"};
  std::vector<std::string> arms_;
  size_t selected_arm_{0};
  bool rotation_mode_{false};
  bool have_joy_{false};
  sensor_msgs::msg::Joy latest_joy_;
  std::vector<int32_t> previous_buttons_;
  std::chrono::steady_clock::time_point last_joy_time_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> publishers_;
  std::unordered_map<std::string, rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr> clients_;
  std::unordered_map<std::string, bool> ready_;
  std::unordered_map<std::string, bool> request_pending_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ServoTwistTeleop>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_twist_teleop"), "Failed to start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
