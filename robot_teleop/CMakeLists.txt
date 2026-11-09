cmake_minimum_required(VERSION 3.8)
project(robot_teleop)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  arm_cartesian_delta_teleop_node
  src/arm_cartesian_delta_teleop_node.cpp
)

ament_target_dependencies(
  arm_cartesian_delta_teleop_node
  geometry_msgs
  rclcpp
  std_msgs
)

add_executable(
  arm_cartesian_teleop_node
  src/arm_cartesian_teleop_node.cpp
)

ament_target_dependencies(
  arm_cartesian_teleop_node
  geometry_msgs
  rclcpp
  std_msgs
)

add_executable(
  hand_teleop_node
  src/hand_teleop_node.cpp
)

ament_target_dependencies(
  hand_teleop_node
  rclcpp
  std_msgs
)

install(
  TARGETS
    arm_cartesian_delta_teleop_node
    arm_cartesian_teleop_node
    hand_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
