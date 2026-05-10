cmake_minimum_required(VERSION 3.8)
project(sobits_teleop)

# require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(urdf REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

add_library(sobits_teleop_components SHARED src/sobits_teleop.cpp)
ament_target_dependencies(
  sobits_teleop_components
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  sobits_interfaces
  urdf
)
rclcpp_components_register_node(sobits_teleop_components
  PLUGIN "sobits_teleop::SOBITSTeleop"
  EXECUTABLE sobits_teleop
)

add_executable(arm_scale_calibrator src/arm_scale_calibrator.cpp)
ament_target_dependencies(
  arm_scale_calibrator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS sobits_teleop_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  sobits_teleop
  arm_scale_calibrator
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include
)
  
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
