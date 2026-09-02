#include <gtest/gtest.h>
#include <cmath>
#include "sobits_teleop/reach_clamp.hpp"

using sobits_teleop::clamp_to_reach;

namespace
{
const tf2::Vector3 kOrigin(0.0, 0.0, 0.0);
const tf2::Vector3 kShoulder(0.1, -0.2, 0.9);
}  // namespace

TEST(ReachClamp, TargetInsideReachIsUnchanged)
{
  tf2::Vector3 target = kOrigin + tf2::Vector3(0.1, 0.0, 0.0);
  auto result = clamp_to_reach(kOrigin, target, 1.0);
  EXPECT_DOUBLE_EQ(result.x(), target.x());
  EXPECT_DOUBLE_EQ(result.y(), target.y());
  EXPECT_DOUBLE_EQ(result.z(), target.z());
}

TEST(ReachClamp, TargetOutsideReachLandsExactlyOnSphere)
{
  tf2::Vector3 target = kOrigin + tf2::Vector3(3.0, 4.0, 0.0);  // dist 5
  auto result = clamp_to_reach(kOrigin, target, 1.0);
  EXPECT_NEAR((result - kOrigin).length(), 1.0, 1e-9);
}

TEST(ReachClamp, ClampedResultPreservesDirection)
{
  tf2::Vector3 target = kOrigin + tf2::Vector3(3.0, 4.0, 0.0);
  auto result = clamp_to_reach(kOrigin, target, 1.0);
  tf2::Vector3 want_dir = (target - kOrigin).normalized();
  tf2::Vector3 got_dir = (result - kOrigin).normalized();
  EXPECT_NEAR(got_dir.x(), want_dir.x(), 1e-9);
  EXPECT_NEAR(got_dir.y(), want_dir.y(), 1e-9);
  EXPECT_NEAR(got_dir.z(), want_dir.z(), 1e-9);
}

TEST(ReachClamp, ZeroMaxReachDisablesClamp)
{
  tf2::Vector3 target = kOrigin + tf2::Vector3(3.0, 4.0, 0.0);
  auto result = clamp_to_reach(kOrigin, target, 0.0);
  EXPECT_DOUBLE_EQ(result.x(), target.x());
  EXPECT_DOUBLE_EQ(result.y(), target.y());
  EXPECT_DOUBLE_EQ(result.z(), target.z());
}

TEST(ReachClamp, NegativeMaxReachDisablesClamp)
{
  tf2::Vector3 target = kOrigin + tf2::Vector3(3.0, 4.0, 0.0);
  auto result = clamp_to_reach(kOrigin, target, -1.0);
  EXPECT_DOUBLE_EQ(result.x(), target.x());
  EXPECT_DOUBLE_EQ(result.y(), target.y());
  EXPECT_DOUBLE_EQ(result.z(), target.z());
}

TEST(ReachClamp, TargetAtOriginIsUnchangedAndFinite)
{
  auto result = clamp_to_reach(kOrigin, kOrigin, 1.0);
  EXPECT_DOUBLE_EQ(result.x(), kOrigin.x());
  EXPECT_DOUBLE_EQ(result.y(), kOrigin.y());
  EXPECT_DOUBLE_EQ(result.z(), kOrigin.z());
  EXPECT_FALSE(std::isnan(result.x()));
  EXPECT_FALSE(std::isnan(result.y()));
  EXPECT_FALSE(std::isnan(result.z()));
}

TEST(ReachClamp, NonOriginShoulderClampsAroundShoulderNotWorldOrigin)
{
  tf2::Vector3 target = kShoulder + tf2::Vector3(3.0, 4.0, 0.0);  // dist 5 from shoulder
  auto result = clamp_to_reach(kShoulder, target, 1.0);
  EXPECT_NEAR((result - kShoulder).length(), 1.0, 1e-9);
  EXPECT_GT((result - kOrigin).length(), 1.0);
}

TEST(ReachClamp, TargetExactlyAtMaxReachIsUnchanged)
{
  tf2::Vector3 target = kOrigin + tf2::Vector3(1.0, 0.0, 0.0);
  auto result = clamp_to_reach(kOrigin, target, 1.0);
  EXPECT_DOUBLE_EQ(result.x(), target.x());
  EXPECT_DOUBLE_EQ(result.y(), target.y());
  EXPECT_DOUBLE_EQ(result.z(), target.z());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
