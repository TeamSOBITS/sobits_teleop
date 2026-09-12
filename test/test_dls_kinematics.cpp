#include <gtest/gtest.h>

#include "sobits_teleop/dls_kinematics_plugin.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>

namespace
{

// 6-DOF chain replicating the sobit_light arm geometry.
const char kUrdf[] = R"(
<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_footprint"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link1"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <link name="link2"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <link name="link3"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <link name="link4"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <link name="link5"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <link name="link6"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <link name="tip_link"><inertial><mass value="0.01"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial></link>

  <joint name="j1" type="revolute">
    <parent link="base_footprint"/><child link="link1"/>
    <origin xyz="0.2 0 0.3" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.05095" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.45" upper="1.46" effort="10" velocity="1"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0.022 0 0.128" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.07" upper="1.23" effort="10" velocity="1"/>
  </joint>
  <joint name="j4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0.0835 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="j5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0.0405 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.61" upper="2.07" effort="10" velocity="1"/>
  </joint>
  <joint name="j6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0.064 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="link6"/><child link="tip_link"/>
    <origin xyz="0.11225 0 0" rpy="0 0 0"/>
  </joint>
</robot>
)";

const char kSrdf[] = R"(
<?xml version="1.0"?>
<robot name="test_arm">
  <group name="arm">
    <chain base_link="base_footprint" tip_link="tip_link"/>
  </group>
</robot>
)";

moveit::core::RobotModelPtr buildTestModel()
{
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(kUrdf);
  auto srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initString(*urdf_model, kSrdf);
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

double positionError(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationError(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  Eigen::Quaterniond qa(a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z);
  Eigen::Quaterniond qb(b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z);
  Eigen::AngleAxisd aa(qa.normalized() * qb.normalized().inverse());
  return std::abs(aa.angle());
}

class DlsKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = buildTestModel();
    node_ = std::make_shared<rclcpp::Node>("test_dls_kinematics_node");
    ASSERT_TRUE(plugin_.initialize(node_, *model_, "arm", "base_footprint", {"tip_link"}, 0.1));
  }

  moveit::core::RobotModelPtr model_;
  rclcpp::Node::SharedPtr node_;
  sobits_teleop::DLSKinematicsPlugin plugin_;
};

}  // namespace

TEST(DlsKinematicsInit, InitializeSucceedsAndReportsJoints)
{
  auto model = buildTestModel();
  auto node = std::make_shared<rclcpp::Node>("test_dls_kinematics_init_node");
  sobits_teleop::DLSKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_footprint", {"tip_link"}, 0.1));

  const auto & names = plugin.getJointNames();
  ASSERT_EQ(names.size(), 6u);
  EXPECT_EQ(names[0], "j1");
  EXPECT_EQ(names[1], "j2");
  EXPECT_EQ(names[2], "j3");
  EXPECT_EQ(names[3], "j4");
  EXPECT_EQ(names[4], "j5");
  EXPECT_EQ(names[5], "j6");
}

TEST_F(DlsKinematicsTest, RoundTripFromPerturbedSeed)
{
  const std::vector<double> q0 = {0.1, -1.1, -0.6, 0.2, 0.8, -0.3};
  std::vector<geometry_msgs::msg::Pose> fk_poses;
  ASSERT_TRUE(plugin_.getPositionFK({"tip_link"}, q0, fk_poses));
  const geometry_msgs::msg::Pose target = fk_poses[0];

  std::vector<double> seed = q0;
  for (auto & v : seed)
    v += 0.1;

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin_.getPositionIK(target, seed, solution, err));
  EXPECT_EQ(err.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  std::vector<geometry_msgs::msg::Pose> result_poses;
  ASSERT_TRUE(plugin_.getPositionFK({"tip_link"}, solution, result_poses));
  EXPECT_LT(positionError(target, result_poses[0]), 1e-3);
  EXPECT_LT(orientationError(target, result_poses[0]), 1e-2);
}

TEST_F(DlsKinematicsTest, SingularSeedApproximateSolutionImproves)
{
  const std::vector<double> seed = {0.0, -1.5708, 0.0, 0.0, 0.0, 0.0};
  std::vector<geometry_msgs::msg::Pose> seed_fk;
  ASSERT_TRUE(plugin_.getPositionFK({"tip_link"}, seed, seed_fk));

  geometry_msgs::msg::Pose target = seed_fk[0];
  target.position.z += 0.01;
  Eigen::Quaterniond q_seed(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
  Eigen::Quaterniond q_delta(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_target = (q_delta * q_seed).normalized();
  target.orientation.x = q_target.x();
  target.orientation.y = q_target.y();
  target.orientation.z = q_target.z();
  target.orientation.w = q_target.w();

  kinematics::KinematicsQueryOptions options;
  options.return_approximate_solution = true;
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin_.searchPositionIK(target, seed, 0.005, solution, err, options));

  ASSERT_EQ(solution.size(), seed.size());
  for (double v : solution)
    ASSERT_TRUE(std::isfinite(v));

  const auto & names = plugin_.getJointNames();
  auto model_bounds = model_->getJointModelGroup("arm")->getActiveJointModelsBounds();
  for (size_t i = 0; i < names.size(); ++i)
  {
    const auto & b = model_bounds[i]->at(0);
    EXPECT_GE(solution[i], b.min_position_ - 1e-6);
    EXPECT_LE(solution[i], b.max_position_ + 1e-6);
  }

  const double max_total_step = 50 * 0.2;
  for (size_t i = 0; i < seed.size(); ++i)
    EXPECT_LE(std::abs(solution[i] - seed[i]), max_total_step);

  std::vector<geometry_msgs::msg::Pose> seed_pose_vec = seed_fk;
  std::vector<geometry_msgs::msg::Pose> solution_fk;
  ASSERT_TRUE(plugin_.getPositionFK({"tip_link"}, solution, solution_fk));
  EXPECT_LT(positionError(target, solution_fk[0]), positionError(target, seed_pose_vec[0]));
}

TEST_F(DlsKinematicsTest, UnreachableTargetFailsWithoutApproximate)
{
  const std::vector<double> seed = {0.1, -1.1, -0.6, 0.2, 0.8, -0.3};
  std::vector<geometry_msgs::msg::Pose> fk_poses;
  ASSERT_TRUE(plugin_.getPositionFK({"tip_link"}, seed, fk_poses));

  geometry_msgs::msg::Pose target = fk_poses[0];
  target.position.x += 1.0;

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  kinematics::KinematicsQueryOptions strict_options;
  strict_options.return_approximate_solution = false;
  EXPECT_FALSE(plugin_.searchPositionIK(target, seed, 0.01, solution, err, strict_options));
  EXPECT_EQ(err.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);

  kinematics::KinematicsQueryOptions approx_options;
  approx_options.return_approximate_solution = true;
  EXPECT_TRUE(plugin_.searchPositionIK(target, seed, 0.01, solution, err, approx_options));
}

TEST_F(DlsKinematicsTest, ConsistencyLimitsAreRespected)
{
  const std::vector<double> q0 = {0.1, -1.1, -0.6, 0.2, 0.8, -0.3};
  std::vector<geometry_msgs::msg::Pose> fk_poses;
  ASSERT_TRUE(plugin_.getPositionFK({"tip_link"}, q0, fk_poses));
  const geometry_msgs::msg::Pose target = fk_poses[0];

  std::vector<double> seed = q0;
  for (auto & v : seed)
    v += 0.02;

  const std::vector<double> consistency_limits(6, 0.05);
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin_.searchPositionIK(target, seed, 0.05, consistency_limits, solution, err));

  for (size_t i = 0; i < seed.size(); ++i)
    EXPECT_LE(std::abs(solution[i] - seed[i]), 0.05 + 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
