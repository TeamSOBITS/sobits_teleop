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
find_package(tf2_eigen REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(urdf REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

add_library(moveit_arm_controller_components SHARED src/moveit_arm_controller.cpp)
ament_target_dependencies(
  moveit_arm_controller_components
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  trajectory_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  control_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)
rclcpp_components_register_node(moveit_arm_controller_components
  PLUGIN "sobits_teleop::MoveitArmController"
  EXECUTABLE moveit_arm_controller
)

add_library(servo_target_bridge_components SHARED src/servo_target_bridge.cpp)
ament_target_dependencies(
  servo_target_bridge_components
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  trajectory_msgs
  std_srvs
  moveit_msgs
  tf2_ros
  tf2_geometry_msgs
)
rclcpp_components_register_node(servo_target_bridge_components
  PLUGIN "sobits_teleop::ServoTargetBridge"
  EXECUTABLE servo_target_bridge
)

add_library(arm_scale_calibrator_components SHARED src/arm_scale_calibrator.cpp)
ament_target_dependencies(
  arm_scale_calibrator_components
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
rclcpp_components_register_node(arm_scale_calibrator_components
  PLUGIN "sobits_teleop::ArmScaleCalibrator"
  EXECUTABLE arm_scale_calibrator
)

install(TARGETS
  sobits_teleop_components
  moveit_arm_controller_components
  servo_target_bridge_components
  arm_scale_calibrator_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  sobits_teleop
  moveit_arm_controller
  servo_target_bridge
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

install(PROGRAMS
  scripts/tracking_test.py
  DESTINATION lib/${PROJECT_NAME}
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_metrics test/test_pose_metrics.cpp)
  target_include_directories(test_pose_metrics PRIVATE include)
  ament_target_dependencies(test_pose_metrics geometry_msgs)

  ament_add_gtest(test_arm_naming test/test_arm_naming.cpp)
  target_include_directories(test_arm_naming PRIVATE include)

  ament_add_gtest(test_reach_clamp test/test_reach_clamp.cpp)
  target_include_directories(test_reach_clamp PRIVATE include)
  ament_target_dependencies(test_reach_clamp tf2_ros)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_node_startup.py)
endif()

ament_package()
