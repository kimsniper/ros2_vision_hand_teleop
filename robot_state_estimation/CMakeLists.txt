cmake_minimum_required(VERSION 3.8)
project(robot_state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_vision_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_compile_features(
    ${PROJECT_NAME}
    INTERFACE cxx_std_17
)

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
    DIRECTORY include/
    DESTINATION include
)

add_executable(
    state_estimator_node
    src/state_estimator_node.cpp
)

target_compile_features(
    state_estimator_node
    PRIVATE cxx_std_17
)

target_link_libraries(
    state_estimator_node
    ${PROJECT_NAME}
)

ament_target_dependencies(
    state_estimator_node
    rclcpp
    robot_vision_msgs
)

install(
    TARGETS
        ${PROJECT_NAME}
        state_estimator_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
    export_${PROJECT_NAME}
    HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_package()
