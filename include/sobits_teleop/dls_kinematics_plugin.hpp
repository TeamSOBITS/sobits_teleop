#ifndef SOBITS_TELEOP__DLS_KINEMATICS_PLUGIN_HPP_
#define SOBITS_TELEOP__DLS_KINEMATICS_PLUGIN_HPP_

#include <moveit/kinematics_base/kinematics_base.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace sobits_teleop
{

// Damped-least-squares (Levenberg-Marquardt) differential IK for short chains
// near singularities. Deterministic, local, no random restarts.
class DLSKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModel & robot_model,
                   const std::string & group_name, const std::string & base_frame,
                   const std::vector<std::string> & tip_frames, double search_discretization) override;

  bool getPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                      std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
                      const kinematics::KinematicsQueryOptions & options =
                        kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                         double timeout, std::vector<double> & solution,
                         moveit_msgs::msg::MoveItErrorCodes & error_code,
                         const kinematics::KinematicsQueryOptions & options =
                           kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                         double timeout, const std::vector<double> & consistency_limits,
                         std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
                         const kinematics::KinematicsQueryOptions & options =
                           kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                         double timeout, std::vector<double> & solution, const IKCallbackFn & solution_callback,
                         moveit_msgs::msg::MoveItErrorCodes & error_code,
                         const kinematics::KinematicsQueryOptions & options =
                           kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
                         double timeout, const std::vector<double> & consistency_limits,
                         std::vector<double> & solution, const IKCallbackFn & solution_callback,
                         moveit_msgs::msg::MoveItErrorCodes & error_code,
                         const kinematics::KinematicsQueryOptions & options =
                           kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string> & link_names, const std::vector<double> & joint_angles,
                      std::vector<geometry_msgs::msg::Pose> & poses) const override;

  const std::vector<std::string> & getJointNames() const override;
  const std::vector<std::string> & getLinkNames() const override;

private:
  bool solve(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & seed, double timeout,
             const std::vector<double> & consistency_limits, std::vector<double> & solution,
             const IKCallbackFn & solution_callback, moveit_msgs::msg::MoveItErrorCodes & error_code,
             const kinematics::KinematicsQueryOptions & options) const;

  // FK/Jacobian in the base frame; q is in GROUP joint order.
  Eigen::Isometry3d fk(const Eigen::VectorXd & q) const;
  Eigen::MatrixXd jacobian(const Eigen::VectorXd & q) const;
  // Smallest singular value of the length-normalised Jacobian at q.
  double minSingularValue(const Eigen::VectorXd & q) const;

  template <typename T>
  T readParam(const std::string & name, const T & default_value) const;

  // Damped pseudo-inverse: A^T (A A^T + lambda2 I)^-1.
  static Eigen::MatrixXd dampedPinv(const Eigen::MatrixXd & a, double lambda2);
  // lambda^2 for a length-normalised Jacobian: damping_min, ramping to damping_max near singularity.
  double damping(const Eigen::MatrixXd & js) const;
  // Bounded nullspace step toward posture_target_ at q (zero when posture_gain_ is 0).
  Eigen::VectorXd postureStep(const Eigen::VectorXd & q) const;
  // One damped step for task error es with Jacobian js (zeroed columns = locked joints).
  Eigen::VectorXd taskStep(const Eigen::MatrixXd & js, const Eigen::VectorXd & es, double lam2) const;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  mutable std::mutex kdl_mutex_;

  std::vector<std::string> joint_names_;  // group order
  std::vector<std::string> link_names_;
  std::vector<int> group_to_chain_;       // group index -> index among chain moving joints
  std::map<std::string, int> segment_index_;

  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;

  int max_iterations_ = 50;
  double position_tolerance_ = 1e-4;
  double orientation_tolerance_ = 1e-3;
  double length_scale_ = 0.3;
  double orientation_weight_ = 1.0;
  bool position_priority_ = false;
  double damping_min_ = 0.01;
  double damping_max_ = 0.1;
  double singular_value_threshold_ = 0.05;
  double max_step_ = 0.2;
  double joint_limit_margin_ = 0.03;
  double min_singular_value_ = 0.005;
  // Hold a saturated joint at its bound and re-solve the others (off = clamp after the fact).
  bool joint_clamping_ = true;
  // Redundant chains: nullspace nudge toward posture_target_, at most posture_step_ per solve.
  double posture_gain_ = 0.0;
  double posture_step_ = 0.005;
  Eigen::VectorXd posture_target_;
};

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__DLS_KINEMATICS_PLUGIN_HPP_
