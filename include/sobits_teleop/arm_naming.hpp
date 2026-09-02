#ifndef SOBITS_TELEOP__ARM_NAMING_HPP_
#define SOBITS_TELEOP__ARM_NAMING_HPP_

#include <string>

namespace sobits_teleop
{

// Robot-specific frame/topic conventions live in YAML as {arm}/{side}
// templates (servo_bridge.naming.*); this is just the substitution engine.
namespace arm_naming
{

// "arm_right" -> "right"; a name without the "arm_" prefix is its own side.
inline std::string side_of(const std::string & arm_name)
{
  return arm_name.rfind("arm_", 0) == 0 ? arm_name.substr(4) : arm_name;
}

// Replaces every "{arm}"/"{side}" in tmpl, left to right. Any other "{...}"
// is left untouched so a typo stays visible in the resulting name.
inline std::string expand(const std::string & tmpl, const std::string & arm_name)
{
  const std::string side = side_of(arm_name);
  std::string out;
  out.reserve(tmpl.size());

  for (size_t i = 0; i < tmpl.size(); ) {
    if (tmpl.compare(i, 5, "{arm}") == 0) {
      out += arm_name;
      i += 5;
    } else if (tmpl.compare(i, 6, "{side}") == 0) {
      out += side;
      i += 6;
    } else {
      out += tmpl[i];
      ++i;
    }
  }
  return out;
}

}  // namespace arm_naming

}  // namespace sobits_teleop

#endif  // SOBITS_TELEOP__ARM_NAMING_HPP_
