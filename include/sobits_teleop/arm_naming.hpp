#ifndef SOBITS_TELEOP__ARM_NAMING_HPP_
#define SOBITS_TELEOP__ARM_NAMING_HPP_

#include <string>

namespace sobits_teleop
{

// Frames, topics and node names all default from the arm name, so an arm that
// follows the convention needs no YAML at all. Overrides live in servo_bridge.
namespace arm_naming
{

// "arm_right" -> "right"; a name without the prefix is its own side.
inline std::string side_of(const std::string & arm_name)
{
  return arm_name.rfind("arm_", 0) == 0 ? arm_name.substr(4) : arm_name;
}

inline std::string target_frame(const std::string & arm_name)
{
  return side_of(arm_name) + "_target_link";
}

inline std::string end_effector_frame(const std::string & arm_name)
{
  return "hand_" + side_of(arm_name) + "_end_effector_link";
}

inline std::string reach_origin_frame(const std::string & arm_name)
{
  return arm_name + "_shoulder_tilt_link";
}

inline std::string servo_node(const std::string & arm_name)
{
  return "servo_" + arm_name;
}

inline std::string enable_topic(const std::string & arm_name)
{
  return arm_name + "/moveit_track_enabled";
}

inline std::string joint_traj_topic(const std::string & arm_name)
{
  return arm_name + "_position_controller/joint_trajectory";
}

// servo publishes status on "~/status" relative to its own node name.
inline std::string status_topic(const std::string & servo_node_name)
{
  return servo_node_name + "/status";
}

}  // namespace arm_naming

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__ARM_NAMING_HPP_
