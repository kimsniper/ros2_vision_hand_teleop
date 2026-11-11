/*
 * Copyright 2026 Mezael Docoy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ROBOT_CONTROL__CARTESIAN_CONTROL_NODE_HPP_
#define ROBOT_CONTROL__CARTESIAN_CONTROL_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class CartesianControlNode : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  using GoalHandleFollowJointTrajectory =
      rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit CartesianControlNode(
      const std::string &node_name = "cartesian_control_node",
      bool create_target_subscription = true)
      : Node(node_name), ik_pending_(false), goal_pending_(false) {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");

    base_frame_ = declare_parameter<std::string>("base_frame", "link_0");

    tip_link_ = declare_parameter<std::string>("tip_link", "ee_link");

    target_topic_ =
        declare_parameter<std::string>("target_topic", "/arm/cartesian_target");

    trajectory_action_ = declare_parameter<std::string>(
        "trajectory_action", "/arm_trajectory_controller/"
                             "follow_joint_trajectory");

    trajectory_duration_ =
        declare_parameter<double>("trajectory_duration", 0.20);

    ik_timeout_ = declare_parameter<double>("ik_timeout", 1.0);

    joint_state_subscription_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CartesianControlNode::jointStateCallback, this,
                      std::placeholders::_1));

    if (create_target_subscription) {
      target_subscription_ =
          create_subscription<geometry_msgs::msg::PoseStamped>(
              target_topic_, 10,
              std::bind(&CartesianControlNode::cartesianTargetCallback, this,
                        std::placeholders::_1));
    }

    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, trajectory_action_);

    RCLCPP_INFO(get_logger(), "Cartesian controller ready: %s -> %s",
                target_topic_.c_str(), trajectory_action_.c_str());
  }

protected:
  static builtin_interfaces::msg::Duration durationMessage(double seconds) {
    builtin_interfaces::msg::Duration message;

    const double clamped_seconds = std::max(0.0, seconds);

    message.sec = static_cast<int32_t>(clamped_seconds);

    message.nanosec = static_cast<uint32_t>(
        (clamped_seconds - static_cast<double>(message.sec)) * 1.0e9);

    return message;
  }

  static const std::vector<std::string> &armJointNames() {
    static const std::vector<std::string> names = {"A1", "A2", "A3", "A4",
                                                   "A5", "A6", "A7"};

    return names;
  }

  void submitCartesianTarget(const geometry_msgs::msg::PoseStamped &message) {
    if (ik_pending_ || goal_pending_) {
      RCLCPP_DEBUG(get_logger(), "Ignoring target while control is busy");
      return;
    }

    if (!completeJointStateAvailable()) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring target until A1-A7 states are available");
      return;
    }

    if (!cartesianTargetIsFinite(message)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite pose target");
      return;
    }

    if (!message.header.frame_id.empty() &&
        message.header.frame_id != base_frame_) {
      RCLCPP_WARN(get_logger(), "Ignoring target in frame '%s'; expected '%s'",
                  message.header.frame_id.c_str(), base_frame_.c_str());
      return;
    }

    if (!ik_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "MoveIt /compute_ik service is not ready");
      return;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

    request->ik_request.group_name = planning_group_;
    request->ik_request.ik_link_name = tip_link_;
    request->ik_request.avoid_collisions = true;

    request->ik_request.pose_stamped = message;
    request->ik_request.pose_stamped.header.frame_id = base_frame_;

    auto &seed_state = request->ik_request.robot_state.joint_state;

    seed_state.header.frame_id = base_frame_;
    seed_state.name = armJointNames();

    for (const auto &name : armJointNames()) {
      seed_state.position.push_back(current_joint_positions_.at(name));
    }

    request->ik_request.timeout = durationMessage(ik_timeout_);

    ik_pending_ = true;

    ik_client_->async_send_request(
        request, std::bind(&CartesianControlNode::ikResponseCallback, this,
                           std::placeholders::_1));
  }

  bool controlBusy() const { return ik_pending_ || goal_pending_; }

  std::string planning_group_;
  std::string base_frame_;
  std::string tip_link_;
  std::string target_topic_;
  std::string trajectory_action_;

  double trajectory_duration_;
  double ik_timeout_;

private:
  void
  jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message) {
    for (std::size_t index = 0;
         index < message->name.size() && index < message->position.size();
         ++index) {
      const auto &name = message->name[index];

      if (std::find(armJointNames().begin(), armJointNames().end(), name) !=
          armJointNames().end()) {
        current_joint_positions_[name] = message->position[index];
      }
    }
  }

  bool completeJointStateAvailable() const {
    for (const auto &name : armJointNames()) {
      if (current_joint_positions_.find(name) ==
          current_joint_positions_.end()) {
        return false;
      }
    }

    return true;
  }

  static bool
  cartesianTargetIsFinite(const geometry_msgs::msg::PoseStamped &message) {
    const std::vector<double> values = {
        message.pose.position.x,    message.pose.position.y,
        message.pose.position.z,    message.pose.orientation.x,
        message.pose.orientation.y, message.pose.orientation.z,
        message.pose.orientation.w};

    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }

    return true;
  }

  void cartesianTargetCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    submitCartesianTarget(*message);
  }

  void ikResponseCallback(
      rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
    ik_pending_ = false;

    moveit_msgs::srv::GetPositionIK::Response::SharedPtr response;

    try {
      response = future.get();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "IK service call failed: %s", error.what());
      return;
    }

    if (response->error_code.val !=
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(get_logger(), "IK failed with error code %d",
                  response->error_code.val);
      return;
    }

    std::map<std::string, double> solution;

    const auto &names = response->solution.joint_state.name;

    const auto &positions = response->solution.joint_state.position;

    for (std::size_t index = 0;
         index < names.size() && index < positions.size(); ++index) {
      solution[names[index]] = positions[index];
    }

    for (const auto &name : armJointNames()) {
      if (solution.find(name) == solution.end()) {
        RCLCPP_ERROR(get_logger(), "IK response is missing joint %s",
                     name.c_str());
        return;
      }
    }

    if (!trajectory_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "Arm trajectory action server is not ready");
      return;
    }

    FollowJointTrajectory::Goal goal;

    goal.trajectory.joint_names = armJointNames();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (const auto &name : armJointNames()) {
      point.positions.push_back(solution.at(name));
    }

    point.time_from_start = durationMessage(trajectory_duration_);

    goal.trajectory.points.push_back(point);

    goal_pending_ = true;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;

    options.goal_response_callback =
        std::bind(&CartesianControlNode::goalResponseCallback, this,
                  std::placeholders::_1);

    options.result_callback = std::bind(&CartesianControlNode::resultCallback,
                                        this, std::placeholders::_1);

    trajectory_client_->async_send_goal(goal, options);
  }

  void goalResponseCallback(
      const GoalHandleFollowJointTrajectory::SharedPtr goal_handle) {
    if (!goal_handle) {
      goal_pending_ = false;

      RCLCPP_ERROR(get_logger(), "Arm trajectory goal was rejected");
      return;
    }

    RCLCPP_INFO(get_logger(), "Arm trajectory goal accepted");
  }

  void resultCallback(
      const GoalHandleFollowJointTrajectory::WrappedResult &wrapped_result) {
    goal_pending_ = false;

    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->error_code ==
            control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL) {
      RCLCPP_INFO(get_logger(), "Arm Cartesian target completed");
      return;
    }

    RCLCPP_ERROR(get_logger(), "Arm trajectory failed");
  }

  std::map<std::string, double> current_joint_positions_;

  bool ik_pending_;
  bool goal_pending_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_subscription_;

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
};

#endif // ROBOT_CONTROL__CARTESIAN_CONTROL_NODE_HPP_
