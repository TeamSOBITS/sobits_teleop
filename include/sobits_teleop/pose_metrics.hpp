#ifndef SOBITS_TELEOP__POSE_METRICS_HPP_
#define SOBITS_TELEOP__POSE_METRICS_HPP_

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

namespace sobits_teleop
{

// Euclidean distance between two pose positions [m].
inline double pose_distance(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Angle [rad] between the two orientations: 2*acos(|<qa, qb>|).
inline double pose_angle(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  double dot = a.orientation.x * b.orientation.x +
    a.orientation.y * b.orientation.y +
    a.orientation.z * b.orientation.z +
    a.orientation.w * b.orientation.w;
  dot = std::min(1.0, std::max(-1.0, std::abs(dot)));
  return 2.0 * std::acos(dot);
}

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__POSE_METRICS_HPP_
