#include "sobits_teleop/dls_kinematics_plugin.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions/exceptions.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace sobits_teleop
{

namespace
{
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  t.linear() = q.normalized().toRotationMatrix();
  return t;
}

geometry_msgs::msg::Pose frameToPose(const KDL::Frame & frame)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.p[0];
  pose.position.y = frame.p[1];
  pose.position.z = frame.p[2];
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  pose.orientation.x = x;
  pose.orientation.y = y;
  pose.orientation.z = z;
  pose.orientation.w = w;
  return pose;
}

Eigen::Isometry3d frameToIsometry(const KDL::Frame & frame)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 3; ++i)
  {
    t.translation()(i) = frame.p[i];
    for (int j = 0; j < 3; ++j)
      t.linear()(i, j) = frame.M(i, j);
  }
  return t;
}
}  // namespace

template <typename T>
T DLSKinematicsPlugin::readParam(const std::string & name, const T & default_value) const
{
  if (!node_)
    return default_value;

  const std::string full_name = "robot_description_kinematics." + group_name_ + "." + name;
  try
  {
    if (!node_->has_parameter(full_name))
      node_->declare_parameter<T>(full_name, default_value);
    return node_->get_parameter(full_name).get_value<T>();
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
  {
    RCLCPP_WARN(node_->get_logger(), "Param '%s' has wrong type (%s), using default", full_name.c_str(), e.what());
    return default_value;
  }
  catch (const rclcpp::ParameterTypeException & e)
  {
    RCLCPP_WARN(node_->get_logger(), "Param '%s' has wrong type (%s), using default", full_name.c_str(), e.what());
    return default_value;
  }
}

bool DLSKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr & node,
                                      const moveit::core::RobotModel & robot_model, const std::string & group_name,
                                      const std::string & base_frame, const std::vector<std::string> & tip_frames,
                                      double search_discretization)
{
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  if (tip_frames_.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Expected exactly one tip frame, got %zu",
                 tip_frames_.size());
    return false;
  }

  const auto * jmg = robot_model.getJointModelGroup(group_name);
  if (!jmg)
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Unknown group '%s'", group_name.c_str());
    return false;
  }
  if (!jmg->getMimicJointModels().empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Mimic joints are not supported");
    return false;
  }
  if (jmg->getVariableCount() != jmg->getActiveJointModelNames().size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Only single-DOF active joints are supported");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), tree))
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Failed to build KDL tree from URDF");
    return false;
  }
  if (!tree.getChain(base_frame_, tip_frames_[0], chain_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Failed to extract KDL chain from '%s' to '%s'",
                 base_frame_.c_str(), tip_frames_[0].c_str());
    return false;
  }

  std::vector<std::string> chain_moving_joint_names;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
      chain_moving_joint_names.push_back(joint.getName());
  }

  const std::vector<std::string> & group_joint_names = jmg->getActiveJointModelNames();
  if (chain_moving_joint_names.size() != group_joint_names.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"),
                 "Chain has %zu moving joints but group '%s' has %zu active joints",
                 chain_moving_joint_names.size(), group_name.c_str(), group_joint_names.size());
    return false;
  }

  joint_names_ = group_joint_names;
  link_names_ = jmg->getLinkModelNames();
  group_to_chain_.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto it =
      std::find(chain_moving_joint_names.begin(), chain_moving_joint_names.end(), joint_names_[i]);
    if (it == chain_moving_joint_names.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("dls_kinematics_plugin"), "Group joint '%s' not found in KDL chain",
                   joint_names_[i].c_str());
      return false;
    }
    group_to_chain_[i] = static_cast<int>(std::distance(chain_moving_joint_names.begin(), it));
  }

  const auto & bounds = jmg->getActiveJointModelsBounds();
  q_min_.resize(joint_names_.size());
  q_max_.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto & b = bounds[i]->at(0);
    if (b.position_bounded_)
    {
      q_min_[i] = b.min_position_;
      q_max_[i] = b.max_position_;
    }
    else
    {
      q_min_[i] = -1e9;
      q_max_[i] = 1e9;
    }
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

  segment_index_.clear();
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
    segment_index_[chain_.getSegment(i).getName()] = static_cast<int>(i);

  max_iterations_ = readParam<int>("max_iterations", 50);
  position_tolerance_ = readParam<double>("position_tolerance", 1e-4);
  orientation_tolerance_ = readParam<double>("orientation_tolerance", 1e-3);
  length_scale_ = readParam<double>("length_scale", 0.3);
  orientation_weight_ = readParam<double>("orientation_weight", 1.0);
  position_priority_ = readParam<bool>("position_priority", false);
  damping_min_ = readParam<double>("damping_min", 0.01);
  damping_max_ = readParam<double>("damping_max", 0.1);
  singular_value_threshold_ = readParam<double>("singular_value_threshold", 0.05);
  max_step_ = readParam<double>("max_step", 0.2);
  joint_limit_margin_ = readParam<double>("joint_limit_margin", 0.03);
  min_singular_value_ = readParam<double>("min_singular_value", 0.005);

  RCLCPP_INFO(rclcpp::get_logger("dls_kinematics_plugin"),
              "DLS IK for '%s': %s -> %s, %zu joints, damping [%.3f, %.3f], max_step=%.3f",
              group_name_.c_str(), base_frame_.c_str(), tip_frames_[0].c_str(), joint_names_.size(), damping_min_,
              damping_max_, max_step_);
  return true;
}

Eigen::Isometry3d DLSKinematicsPlugin::fk(const Eigen::VectorXd & q) const
{
  KDL::JntArray kdl_q(chain_.getNrOfJoints());
  for (size_t i = 0; i < joint_names_.size(); ++i)
    kdl_q(group_to_chain_[i]) = q[static_cast<int>(i)];

  KDL::Frame frame;
  std::lock_guard<std::mutex> lock(kdl_mutex_);
  fk_solver_->JntToCart(kdl_q, frame);
  return frameToIsometry(frame);
}

Eigen::MatrixXd DLSKinematicsPlugin::jacobian(const Eigen::VectorXd & q) const
{
  KDL::JntArray kdl_q(chain_.getNrOfJoints());
  for (size_t i = 0; i < joint_names_.size(); ++i)
    kdl_q(group_to_chain_[i]) = q[static_cast<int>(i)];

  KDL::Jacobian kdl_jac(chain_.getNrOfJoints());
  {
    std::lock_guard<std::mutex> lock(kdl_mutex_);
    jac_solver_->JntToJac(kdl_q, kdl_jac);
  }

  Eigen::MatrixXd j(6, static_cast<int>(joint_names_.size()));
  for (size_t i = 0; i < joint_names_.size(); ++i)
    j.col(static_cast<int>(i)) = kdl_jac.data.col(group_to_chain_[i]);
  return j;
}

double DLSKinematicsPlugin::minSingularValue(const Eigen::VectorXd & q) const
{
  Eigen::MatrixXd js = jacobian(q);
  js.topRows(3) /= length_scale_;
  return Eigen::JacobiSVD<Eigen::MatrixXd>(js).singularValues().minCoeff();
}

Eigen::MatrixXd DLSKinematicsPlugin::dampedPinv(const Eigen::MatrixXd & a, double lambda2)
{
  const Eigen::MatrixXd aat = a * a.transpose();
  const Eigen::MatrixXd damped = aat + lambda2 * Eigen::MatrixXd::Identity(aat.rows(), aat.cols());
  return a.transpose() * damped.ldlt().solve(Eigen::MatrixXd::Identity(damped.rows(), damped.cols()));
}

bool DLSKinematicsPlugin::solve(const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & seed,
                                 double timeout, const std::vector<double> & consistency_limits,
                                 std::vector<double> & solution, const IKCallbackFn & solution_callback,
                                 moveit_msgs::msg::MoveItErrorCodes & error_code,
                                 const kinematics::KinematicsQueryOptions & options) const
{
  const auto n = static_cast<Eigen::Index>(joint_names_.size());
  if (static_cast<Eigen::Index>(seed.size()) != n)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  Eigen::VectorXd seed_v = Eigen::Map<const Eigen::VectorXd>(seed.data(), n);
  Eigen::VectorXd lo(n), hi(n);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    lo[i] = q_min_[i] + joint_limit_margin_;
    hi[i] = q_max_[i] - joint_limit_margin_;
    if (lo[i] > hi[i])
    {
      const double mid = 0.5 * (q_min_[i] + q_max_[i]);
      lo[i] = hi[i] = mid;
    }
  }
  if (static_cast<Eigen::Index>(consistency_limits.size()) == n)
  {
    for (Eigen::Index i = 0; i < n; ++i)
    {
      lo[i] = std::max(lo[i], seed_v[i] - consistency_limits[static_cast<size_t>(i)]);
      hi[i] = std::min(hi[i], seed_v[i] + consistency_limits[static_cast<size_t>(i)]);
    }
  }

  const Eigen::VectorXd q0 = seed_v.cwiseMax(lo).cwiseMin(hi);
  Eigen::VectorXd q = q0;
  const Eigen::Isometry3d t_d = poseToIsometry(ik_pose);

  const auto start = std::chrono::steady_clock::now();
  bool converged = false;

  for (int it = 0; it < max_iterations_; ++it)
  {
    const Eigen::Isometry3d t = fk(q);
    const Eigen::Vector3d e_p = t_d.translation() - t.translation();
    const Eigen::AngleAxisd e_o_aa(t_d.linear() * t.linear().transpose());
    const Eigen::Vector3d e_o = e_o_aa.angle() * e_o_aa.axis();

    if (e_p.norm() < position_tolerance_ && e_o.norm() < orientation_tolerance_)
    {
      converged = true;
      break;
    }
    if (timeout > 0.0)
    {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed > timeout)
        break;
    }

    Eigen::MatrixXd js = jacobian(q);
    js.topRows(3) /= length_scale_;
    Eigen::VectorXd es(6);
    es.head<3>() = e_p / length_scale_;
    es.tail<3>() = e_o;

    const double s_min = Eigen::JacobiSVD<Eigen::MatrixXd>(js).singularValues().minCoeff();
    double lam2 = damping_min_ * damping_min_;
    if (s_min < singular_value_threshold_)
    {
      const double ratio = 1.0 - s_min / singular_value_threshold_;
      lam2 += damping_max_ * damping_max_ * ratio * ratio;
    }

    Eigen::VectorXd dq(n);
    if (!position_priority_)
    {
      Eigen::VectorXd w(6);
      w << 1.0, 1.0, 1.0, orientation_weight_, orientation_weight_, orientation_weight_;
      const Eigen::MatrixXd jtw = js.transpose() * w.asDiagonal();
      const Eigen::MatrixXd h = jtw * js + lam2 * Eigen::MatrixXd::Identity(n, n);
      dq = h.ldlt().solve(jtw * es);
    }
    else
    {
      const Eigen::MatrixXd jp = js.topRows(3);
      const Eigen::MatrixXd jo = js.bottomRows(3);
      const Eigen::Vector3d es_p = es.head<3>();
      const Eigen::Vector3d es_o = es.tail<3>();

      const Eigen::MatrixXd pinv_p = dampedPinv(jp, lam2);
      const Eigen::VectorXd dq_p = pinv_p * es_p;
      const Eigen::MatrixXd np = Eigen::MatrixXd::Identity(n, n) - pinv_p * jp;

      const Eigen::MatrixXd jo_np = jo * np;
      const Eigen::MatrixXd pinv_o = dampedPinv(jo_np, lam2);
      dq = dq_p + pinv_o * (es_o - jo * dq_p);
    }

    dq = dq.cwiseMax(-max_step_).cwiseMin(max_step_);
    q = (q + dq).cwiseMax(lo).cwiseMin(hi);
  }

  if (!q.allFinite())
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  // An unreachable target makes the iteration converge onto the singular surface
  // itself; never return a state deeper into it than the floor (or the seed).
  if (min_singular_value_ > 0.0)
  {
    const double s_floor = std::min(min_singular_value_, minSingularValue(q0));
    if (minSingularValue(q) < s_floor)
    {
      const Eigen::VectorXd step = q - q0;
      double t_lo = 0.0, t_hi = 1.0;
      for (int k = 0; k < 8; ++k)
      {
        const double t = 0.5 * (t_lo + t_hi);
        if (minSingularValue(q0 + t * step) >= s_floor)
          t_lo = t;
        else
          t_hi = t;
      }
      q = q0 + t_lo * step;
      converged = false;
    }
  }
  if (!converged && !options.return_approximate_solution)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution.assign(q.data(), q.data() + q.size());
  if (solution_callback)
  {
    solution_callback(ik_pose, solution, error_code);
    if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      return false;
  }
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool DLSKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                                         const std::vector<double> & ik_seed_state, std::vector<double> & solution,
                                         moveit_msgs::msg::MoveItErrorCodes & error_code,
                                         const kinematics::KinematicsQueryOptions & options) const
{
  return solve(ik_pose, ik_seed_state, 0.0, {}, solution, IKCallbackFn(), error_code, options);
}

bool DLSKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                                            const std::vector<double> & ik_seed_state, double timeout,
                                            std::vector<double> & solution,
                                            moveit_msgs::msg::MoveItErrorCodes & error_code,
                                            const kinematics::KinematicsQueryOptions & options) const
{
  return solve(ik_pose, ik_seed_state, timeout, {}, solution, IKCallbackFn(), error_code, options);
}

bool DLSKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                                            const std::vector<double> & ik_seed_state, double timeout,
                                            const std::vector<double> & consistency_limits,
                                            std::vector<double> & solution,
                                            moveit_msgs::msg::MoveItErrorCodes & error_code,
                                            const kinematics::KinematicsQueryOptions & options) const
{
  return solve(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool DLSKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                                            const std::vector<double> & ik_seed_state, double timeout,
                                            std::vector<double> & solution, const IKCallbackFn & solution_callback,
                                            moveit_msgs::msg::MoveItErrorCodes & error_code,
                                            const kinematics::KinematicsQueryOptions & options) const
{
  return solve(ik_pose, ik_seed_state, timeout, {}, solution, solution_callback, error_code, options);
}

bool DLSKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose & ik_pose,
                                            const std::vector<double> & ik_seed_state, double timeout,
                                            const std::vector<double> & consistency_limits,
                                            std::vector<double> & solution, const IKCallbackFn & solution_callback,
                                            moveit_msgs::msg::MoveItErrorCodes & error_code,
                                            const kinematics::KinematicsQueryOptions & options) const
{
  return solve(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code, options);
}

bool DLSKinematicsPlugin::getPositionFK(const std::vector<std::string> & link_names,
                                         const std::vector<double> & joint_angles,
                                         std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (static_cast<size_t>(joint_angles.size()) != joint_names_.size())
    return false;

  KDL::JntArray kdl_q(chain_.getNrOfJoints());
  for (size_t i = 0; i < joint_names_.size(); ++i)
    kdl_q(group_to_chain_[i]) = joint_angles[i];

  poses.resize(link_names.size());
  for (size_t i = 0; i < link_names.size(); ++i)
  {
    const auto it = segment_index_.find(link_names[i]);
    if (it == segment_index_.end())
      return false;

    KDL::Frame frame;
    std::lock_guard<std::mutex> lock(kdl_mutex_);
    if (fk_solver_->JntToCart(kdl_q, frame, it->second + 1) < 0)
      return false;
    poses[i] = frameToPose(frame);
  }
  return true;
}

const std::vector<std::string> & DLSKinematicsPlugin::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string> & DLSKinematicsPlugin::getLinkNames() const
{
  return link_names_;
}

}  // namespace sobits_teleop

PLUGINLIB_EXPORT_CLASS(sobits_teleop::DLSKinematicsPlugin, kinematics::KinematicsBase)
