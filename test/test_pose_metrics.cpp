#include <gtest/gtest.h>
#include "sobits_teleop/pose_metrics.hpp"

namespace
{

// Identity pose: origin position, identity orientation.
geometry_msgs::msg::Pose identity_pose()
{
  geometry_msgs::msg::Pose p;
  p.orientation.w = 1.0;
  return p;
}

// Pose rotated by angle_rad about the given axis (unit vector assumed).
geometry_msgs::msg::Pose axis_rotated_pose(double x, double y, double z, double angle_rad)
{
  geometry_msgs::msg::Pose p;
  double half = angle_rad / 2.0;
  p.orientation.x = x * std::sin(half);
  p.orientation.y = y * std::sin(half);
  p.orientation.z = z * std::sin(half);
  p.orientation.w = std::cos(half);
  return p;
}

}  // namespace

TEST(PoseMetrics, IdenticalPosesHaveZeroDistance)
{
  auto p = identity_pose();
  EXPECT_DOUBLE_EQ(sobits_teleop::pose_distance(p, p), 0.0);
}

TEST(PoseMetrics, IdenticalPosesHaveZeroAngle)
{
  auto p = identity_pose();
  EXPECT_NEAR(sobits_teleop::pose_angle(p, p), 0.0, 1e-9);
}

TEST(PoseMetrics, KnownThreeFourFiveDistance)
{
  auto a = identity_pose();
  auto b = identity_pose();
  b.position.x = 3.0;
  b.position.y = 4.0;
  b.position.z = 0.0;
  EXPECT_DOUBLE_EQ(sobits_teleop::pose_distance(a, b), 5.0);
}

TEST(PoseMetrics, NinetyDegreeRotationAboutZ)
{
  auto a = identity_pose();
  auto b = axis_rotated_pose(0.0, 0.0, 1.0, M_PI / 2.0);
  EXPECT_NEAR(sobits_teleop::pose_angle(a, b), M_PI / 2.0, 1e-9);
}

TEST(PoseMetrics, OneEightyDegreeRotationAboutZ)
{
  auto a = identity_pose();
  auto b = axis_rotated_pose(0.0, 0.0, 1.0, M_PI);
  EXPECT_NEAR(sobits_teleop::pose_angle(a, b), M_PI, 1e-9);
}

TEST(PoseMetrics, QuaternionDoubleCoverGivesZeroAngle)
{
  auto a = axis_rotated_pose(0.0, 1.0, 0.0, 1.234);
  auto b = a;
  // Negate all components: same rotation, opposite sign (double cover).
  b.orientation.x = -b.orientation.x;
  b.orientation.y = -b.orientation.y;
  b.orientation.z = -b.orientation.z;
  b.orientation.w = -b.orientation.w;
  EXPECT_NEAR(sobits_teleop::pose_angle(a, b), 0.0, 1e-9);
}

TEST(PoseMetrics, NonNormalizedQuaternionsClampWithoutNaN)
{
  geometry_msgs::msg::Pose a;
  a.orientation.x = 0.0;
  a.orientation.y = 0.0;
  a.orientation.z = 0.0;
  a.orientation.w = 2.0;  // not normalized — dot product would exceed 1.0
  geometry_msgs::msg::Pose b;
  b.orientation.x = 0.0;
  b.orientation.y = 0.0;
  b.orientation.z = 0.0;
  b.orientation.w = 2.0;
  double angle = sobits_teleop::pose_angle(a, b);
  EXPECT_FALSE(std::isnan(angle));
  EXPECT_NEAR(angle, 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
