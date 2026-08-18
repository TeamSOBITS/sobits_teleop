#include <gtest/gtest.h>
#include "sobits_teleop/arm_naming.hpp"

using sobits_teleop::arm_naming::side_of;
using sobits_teleop::arm_naming::expand;

TEST(ArmNaming, SideOfStripsArmPrefix)
{
  EXPECT_EQ(side_of("arm_right"), "right");
  EXPECT_EQ(side_of("arm_left"), "left");
}

TEST(ArmNaming, SideOfWithoutPrefixReturnsUnchanged)
{
  EXPECT_EQ(side_of("gripper"), "gripper");
}

TEST(ArmNaming, SideOfDegeneratePrefixOnlyReturnsEmpty)
{
  EXPECT_EQ(side_of("arm_"), "");
}

TEST(ArmNaming, ExpandArmPlaceholder)
{
  EXPECT_EQ(expand("{arm}_shoulder_tilt_link", "arm_right"), "arm_right_shoulder_tilt_link");
}

TEST(ArmNaming, ExpandSidePlaceholder)
{
  EXPECT_EQ(expand("{side}_target_link", "arm_right"), "right_target_link");
}

TEST(ArmNaming, ExpandBothPlaceholdersInOneTemplate)
{
  EXPECT_EQ(expand("{arm}/{side}", "arm_left"), "arm_left/left");
}

TEST(ArmNaming, ExpandRepeatedPlaceholders)
{
  EXPECT_EQ(expand("{side}-{side}", "arm_right"), "right-right");
}

TEST(ArmNaming, ExpandNoPlaceholdersReturnsUnchanged)
{
  EXPECT_EQ(expand("base_footprint", "arm_right"), "base_footprint");
}

TEST(ArmNaming, ExpandUnknownPlaceholderLeftIntact)
{
  EXPECT_EQ(expand("{bogus}_link", "arm_right"), "{bogus}_link");
}

// {servo_node} is resolved by the bridge (a second targeted replacement),
// not by expand() — confirm expand() itself leaves it untouched.
TEST(ArmNaming, ExpandLeavesServoNodePlaceholderUntouched)
{
  EXPECT_EQ(expand("{servo_node}/status", "arm_right"), "{servo_node}/status");
}

TEST(ArmNaming, ExpandEmptyTemplateReturnsEmpty)
{
  EXPECT_EQ(expand("", "arm_right"), "");
}

// Regression guard: expanding each of the seven default templates from
// arm_backend_servo.yaml must exactly match the old hardcoded functions.
TEST(ArmNaming, DefaultTemplatesMatchOldHardcodedBehaviourRight)
{
  EXPECT_EQ(expand("{side}_target_link", "arm_right"), "right_target_link");
  EXPECT_EQ(
    expand("hand_{side}_end_effector_link", "arm_right"), "hand_right_end_effector_link");
  EXPECT_EQ(expand("{arm}_shoulder_tilt_link", "arm_right"), "arm_right_shoulder_tilt_link");
  EXPECT_EQ(expand("servo_{arm}", "arm_right"), "servo_arm_right");
  EXPECT_EQ(expand("{arm}/moveit_track_enabled", "arm_right"), "arm_right/moveit_track_enabled");
  EXPECT_EQ(
    expand("{arm}_position_controller/joint_trajectory", "arm_right"),
    "arm_right_position_controller/joint_trajectory");
  // status_topic's {servo_node} is resolved by the bridge, not by expand()
  // (see servo_target_bridge.cpp); {arm}/{side} substitution doesn't apply.
}

TEST(ArmNaming, DefaultTemplatesMatchOldHardcodedBehaviourLeft)
{
  EXPECT_EQ(expand("{side}_target_link", "arm_left"), "left_target_link");
  EXPECT_EQ(
    expand("hand_{side}_end_effector_link", "arm_left"), "hand_left_end_effector_link");
  EXPECT_EQ(expand("{arm}_shoulder_tilt_link", "arm_left"), "arm_left_shoulder_tilt_link");
  EXPECT_EQ(expand("servo_{arm}", "arm_left"), "servo_arm_left");
  EXPECT_EQ(expand("{arm}/moveit_track_enabled", "arm_left"), "arm_left/moveit_track_enabled");
  EXPECT_EQ(
    expand("{arm}_position_controller/joint_trajectory", "arm_left"),
    "arm_left_position_controller/joint_trajectory");
}

TEST(ArmNaming, NonConventionalNameProducesWellFormedStrings)
{
  EXPECT_EQ(expand("{side}_target_link", "gripper"), "gripper_target_link");
  EXPECT_EQ(expand("hand_{side}_end_effector_link", "gripper"), "hand_gripper_end_effector_link");
  EXPECT_EQ(expand("{arm}_shoulder_tilt_link", "gripper"), "gripper_shoulder_tilt_link");
  EXPECT_EQ(expand("servo_{arm}", "gripper"), "servo_gripper");
  EXPECT_EQ(expand("{arm}/moveit_track_enabled", "gripper"), "gripper/moveit_track_enabled");
  EXPECT_EQ(
    expand("{arm}_position_controller/joint_trajectory", "gripper"),
    "gripper_position_controller/joint_trajectory");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
