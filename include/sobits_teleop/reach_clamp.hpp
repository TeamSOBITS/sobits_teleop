#ifndef SOBITS_TELEOP__REACH_CLAMP_HPP_
#define SOBITS_TELEOP__REACH_CLAMP_HPP_

#include <tf2/LinearMath/Vector3.h>

namespace sobits_teleop
{

// Pulls an out-of-reach target onto the max_reach sphere around origin.
// Returns the target unchanged when max_reach <= 0 or it is already in reach.
inline tf2::Vector3 clamp_to_reach(
  const tf2::Vector3 & origin, const tf2::Vector3 & target, double max_reach)
{
  if (max_reach <= 0.0) {
    return target;
  }
  tf2::Vector3 offset = target - origin;
  const double dist = offset.length();
  if (dist > max_reach) {
    return origin + offset * (max_reach / dist);
  }
  return target;
}

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__REACH_CLAMP_HPP_
