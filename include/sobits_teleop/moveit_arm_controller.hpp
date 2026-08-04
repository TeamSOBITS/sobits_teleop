#ifndef SOBITS_TELEOP__MOVEIT_ARM_CONTROLLER_HPP_
#define SOBITS_TELEOP__MOVEIT_ARM_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace sobits_teleop
{

struct ArmTeleopConfig {
  std::string planning_group;
  std::string target_frame;
  std::string base_frame;
  std::string trajectory_topic;
};

class MoveitArmController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit MoveitArmController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MoveitArmController();

private:
  struct ArmData {
    ArmTeleopConfig config;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub;
    std::shared_ptr<GoalHandleFJT> goal_handle;  // protected by goal_mutex
    std::mutex goal_mutex;
    std::atomic<bool> executing{false};
    std::atomic<bool> enabled{false};
    std::atomic<bool> thread_active{false};
    std::thread thread;

    std::unordered_map<std::string, double> vel_limits;
    std::unordered_map<std::string, double> accel_limits;

    // Streaming seed: last sent trajectory + send time, so the next hop plans from
    // the commanded state. Guarded by last_sent_mutex (pointer swap/read only).
    robot_trajectory::RobotTrajectoryPtr last_sent_traj;
    rclcpp::Time last_sent_time;
    std::mutex last_sent_mutex;

    // Only this arm's own tracking thread touches it.
    double last_heartbeat_sec{0.0};

    ArmData() = default;
    ArmData(const ArmData &) = delete;
    ArmData & operator=(const ArmData &) = delete;
  };

  void init_move_groups();

  void enable_callback(
    const std::string & arm_name,
    const std_msgs::msg::Bool::SharedPtr msg);

  void tracking_loop(const std::string & arm_name);

  void send_trajectory(ArmData & arm, const trajectory_msgs::msg::JointTrajectory & jtraj);
  void cancel_trajectory(ArmData & arm);

  static double pose_distance(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b);

  static double pose_angle(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b);

  std::thread init_thread_;

  std::unordered_map<std::string, std::unique_ptr<ArmData>> arms_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> enable_subs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // The arms share one RobotModel and MoveIt planning isn't thread-safe across it.
  // Held around the compute section only, released before the send.
  std::mutex planning_mutex_;

  double update_rate_hz_;
  double max_cartesian_step_m_;
  double min_cartesian_fraction_;
  double arrival_threshold_m_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double eef_step_m_;
  double replan_threshold_m_;
  int    traj_lookahead_ms_;
  double ompl_planning_timeout_s_;
  double preempt_threshold_m_;
  double arrival_threshold_rad_;
  double replan_threshold_rad_;
  double preempt_threshold_rad_;
  bool   avoid_collisions_;
  int    preempt_settle_ms_;
  bool   use_topic_;

  static constexpr double heartbeat_period_sec_ = 2.0;
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__MOVEIT_ARM_CONTROLLER_HPP_
