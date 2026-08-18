#include <gtest/gtest.h>
#include "sobits_teleop/arm_naming.hpp"

using sobits_teleop::arm_naming::side_of;
using sobits_teleop::arm_naming::target_frame;
using sobits_teleop::arm_naming::end_effector_frame;
using sobits_teleop::arm_naming::reach_origin_frame;
using sobits_teleop::arm_naming::servo_node;
using sobits_teleop::arm_naming::enable_topic;
using sobits_teleop::arm_naming::joint_traj_topic;
using sobits_teleop::arm_naming::status_topic;

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

TEST(ArmNaming, TargetFrame)
{
  EXPECT_EQ(target_frame("arm_right"), "right_target_link");
  EXPECT_EQ(target_frame("arm_left"), "left_target_link");
}

TEST(ArmNaming, EndEffectorFrame)
{
  EXPECT_EQ(end_effector_frame("arm_right"), "hand_right_end_effector_link");
  EXPECT_EQ(end_effector_frame("arm_left"), "hand_left_end_effector_link");
}

TEST(ArmNaming, ReachOriginFrame)
{
  EXPECT_EQ(reach_origin_frame("arm_right"), "arm_right_shoulder_tilt_link");
  EXPECT_EQ(reach_origin_frame("arm_left"), "arm_left_shoulder_tilt_link");
}

TEST(ArmNaming, ServoNode)
{
  EXPECT_EQ(servo_node("arm_right"), "servo_arm_right");
  EXPECT_EQ(servo_node("arm_left"), "servo_arm_left");
}

TEST(ArmNaming, EnableTopic)
{
  EXPECT_EQ(enable_topic("arm_right"), "arm_right/moveit_track_enabled");
  EXPECT_EQ(enable_topic("arm_left"), "arm_left/moveit_track_enabled");
}

TEST(ArmNaming, JointTrajTopic)
{
  EXPECT_EQ(joint_traj_topic("arm_right"), "arm_right_position_controller/joint_trajectory");
  EXPECT_EQ(joint_traj_topic("arm_left"), "arm_left_position_controller/joint_trajectory");
}

TEST(ArmNaming, StatusTopic)
{
  EXPECT_EQ(status_topic("servo_arm_right"), "servo_arm_right/status");
}

TEST(ArmNaming, NonConventionalNameProducesWellFormedStrings)
{
  EXPECT_EQ(target_frame("gripper"), "gripper_target_link");
  EXPECT_EQ(end_effector_frame("gripper"), "hand_gripper_end_effector_link");
  EXPECT_EQ(reach_origin_frame("gripper"), "gripper_shoulder_tilt_link");
  EXPECT_EQ(servo_node("gripper"), "servo_gripper");
  EXPECT_EQ(enable_topic("gripper"), "gripper/moveit_track_enabled");
  EXPECT_EQ(joint_traj_topic("gripper"), "gripper_position_controller/joint_trajectory");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
