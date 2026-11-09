cmake_minimum_required(VERSION 3.8)
project(robot_control)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(
  cartesian_control_node
  src/cartesian_control_node.cpp
)

target_include_directories(
  cartesian_control_node
  PRIVATE include
)

ament_target_dependencies(
  cartesian_control_node
  builtin_interfaces
  control_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

add_executable(
  cartesian_delta_control_node
  src/cartesian_delta_control_node.cpp
)

target_include_directories(
  cartesian_delta_control_node
  PRIVATE include
)

ament_target_dependencies(
  cartesian_delta_control_node
  builtin_interfaces
  control_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
  trajectory_msgs
)

install(
  TARGETS
    cartesian_delta_control_node
    cartesian_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
