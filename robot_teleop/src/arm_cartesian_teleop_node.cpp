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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_vision_msgs/msg/hand_state.hpp"

class ArmCartesianTeleopNode : public rclcpp::Node {
public:
  using Point = std::array<double, 3>;

  ArmCartesianTeleopNode()
      : Node("arm_cartesian_teleop_node"), latest_palm_center_{0.0, 0.0, 0.0},
        calibrated_palm_center_{0.0, 0.0, 0.0}, reacquisition_palm_sum_{0.0,
                                                                        0.0,
                                                                        0.0},
        latest_relative_depth_(0.0), reacquisition_sample_count_(0),
        has_latest_palm_center_(false), depth_valid_(false),
        calibrated_(false) {
    const auto state_topic =
        declare_parameter<std::string>("state_topic", "/hand/state");

    const auto target_topic =
        declare_parameter<std::string>("target_topic", "/arm/cartesian_target");

    base_frame_ = declare_parameter<std::string>("base_frame", "link_0");

    tip_link_ = declare_parameter<std::string>("tip_link", "ee_link");

    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);

    workspace_x_scale_ = declare_parameter<double>("workspace_x_scale", 0.30);

    workspace_y_scale_ = declare_parameter<double>("workspace_y_scale", 0.60);

    workspace_z_scale_ = declare_parameter<double>("workspace_z_scale", 0.50);

    maximum_x_offset_ = declare_parameter<double>("maximum_x_offset", 0.15);

    reacquisition_samples_ = declare_parameter<int>("reacquisition_samples", 5);

    maximum_y_offset_ = declare_parameter<double>("maximum_y_offset", 0.25);

    maximum_z_offset_ = declare_parameter<double>("maximum_z_offset", 0.20);

    minimum_target_change_ =
        declare_parameter<double>("minimum_target_change", 0.005);

    reference_x_ = declare_parameter<double>("reference_x", 0.615);

    reference_y_ = declare_parameter<double>("reference_y", 0.000);

    reference_z_ = declare_parameter<double>("reference_z", 0.975);

    reference_orientation_x_ =
        declare_parameter<double>("reference_orientation_x", 0.0000000);

    reference_orientation_y_ =
        declare_parameter<double>("reference_orientation_y", -0.3826834);

    reference_orientation_z_ =
        declare_parameter<double>("reference_orientation_z", 0.0000000);

    reference_orientation_w_ =
        declare_parameter<double>("reference_orientation_w", 0.9238795);

    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    if (workspace_x_scale_ <= 0.0) {
      throw std::invalid_argument("workspace_x_scale must be positive");
    }

    if (maximum_x_offset_ <= 0.0) {
      throw std::invalid_argument("maximum_x_offset must be positive");
    }

    if (reacquisition_samples_ <= 0) {
      throw std::invalid_argument("reacquisition_samples must be positive");
    }

    target_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, 10);

    state_subscription_ =
        create_subscription<robot_vision_msgs::msg::HandState>(
            state_topic, 10,
            std::bind(&ArmCartesianTeleopNode::stateCallback, this,
                      std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz),
        std::bind(&ArmCartesianTeleopNode::publishTarget, this));

    RCLCPP_INFO(get_logger(), "3D Cartesian arm teleop ready: %s -> %s",
                state_topic.c_str(), target_topic.c_str());
  }

private:
  void resetReacquisition() {
    reacquisition_palm_sum_ = {0.0, 0.0, 0.0};
    reacquisition_sample_count_ = 0;
    has_latest_palm_center_ = false;
  }

  void resetTracking() {
    latest_relative_depth_ = 0.0;
    depth_valid_ = false;
    calibrated_ = false;
    resetReacquisition();
  }

  void
  stateCallback(const robot_vision_msgs::msg::HandState::SharedPtr message) {
    if (!message->hand_detected) {
      resetTracking();
      return;
    }

    if (!message->depth_valid || !std::isfinite(message->relative_depth) ||
        !std::isfinite(message->palm_center.x) ||
        !std::isfinite(message->palm_center.y) ||
        !std::isfinite(message->palm_center.z)) {
      resetTracking();
      return;
    }

    const Point palm_center = {
        message->palm_center.x,
        message->palm_center.y,
        message->palm_center.z,
    };

    latest_relative_depth_ = message->relative_depth;
    depth_valid_ = true;

    if (!calibrated_ && !has_latest_palm_center_) {
      for (std::size_t axis = 0; axis < palm_center.size(); ++axis) {
        reacquisition_palm_sum_[axis] += palm_center[axis];
      }

      ++reacquisition_sample_count_;

      if (reacquisition_sample_count_ < reacquisition_samples_) {
        return;
      }

      for (std::size_t axis = 0; axis < palm_center.size(); ++axis) {
        latest_palm_center_[axis] =
            reacquisition_palm_sum_[axis] /
            static_cast<double>(reacquisition_sample_count_);
      }

      has_latest_palm_center_ = true;

      RCLCPP_INFO(get_logger(),
                  "Stable Cartesian calibration acquired from %d states",
                  reacquisition_sample_count_);

      return;
    }

    latest_palm_center_ = palm_center;
    has_latest_palm_center_ = true;
  }

  bool calibrate() {
    calibrated_palm_center_ = latest_palm_center_;

    calibrated_pose_.header.frame_id = base_frame_;

    calibrated_pose_.pose.position.x = reference_x_;

    calibrated_pose_.pose.position.y = reference_y_;

    calibrated_pose_.pose.position.z = reference_z_;

    calibrated_pose_.pose.orientation.x = reference_orientation_x_;

    calibrated_pose_.pose.orientation.y = reference_orientation_y_;

    calibrated_pose_.pose.orientation.z = reference_orientation_z_;

    calibrated_pose_.pose.orientation.w = reference_orientation_w_;

    last_target_ = calibrated_pose_;
    calibrated_ = true;
    reacquisition_palm_sum_ = {0.0, 0.0, 0.0};
    reacquisition_sample_count_ = 0;

    RCLCPP_INFO(get_logger(),
                "3D Cartesian arm teleop calibrated at "
                "palm=(%.3f, %.3f), depth=%.3f, "
                "reference=(%.3f, %.3f, %.3f)",
                calibrated_palm_center_[0], calibrated_palm_center_[1],
                latest_relative_depth_, reference_x_, reference_y_,
                reference_z_);

    return true;
  }

  void publishTarget() {
    if (!has_latest_palm_center_ || !depth_valid_) {
      return;
    }

    if (!calibrated_ && !calibrate()) {
      return;
    }

    const double palm_offset_x =
        latest_palm_center_[0] - calibrated_palm_center_[0];

    const double palm_offset_y =
        latest_palm_center_[1] - calibrated_palm_center_[1];

    const double x_offset =
        std::clamp(latest_relative_depth_ * workspace_x_scale_,
                   -maximum_x_offset_, maximum_x_offset_);

    const double y_offset = std::clamp(-palm_offset_x * workspace_y_scale_,
                                       -maximum_y_offset_, maximum_y_offset_);

    const double z_offset = std::clamp(-palm_offset_y * workspace_z_scale_,
                                       -maximum_z_offset_, maximum_z_offset_);

    geometry_msgs::msg::PoseStamped target = calibrated_pose_;

    target.header.stamp = now();

    target.pose.position.x = calibrated_pose_.pose.position.x + x_offset;

    target.pose.position.y = calibrated_pose_.pose.position.y + y_offset;

    target.pose.position.z = calibrated_pose_.pose.position.z + z_offset;

    const double x_change =
        std::abs(target.pose.position.x - last_target_.pose.position.x);

    const double y_change =
        std::abs(target.pose.position.y - last_target_.pose.position.y);

    const double z_change =
        std::abs(target.pose.position.z - last_target_.pose.position.z);

    if (x_change < minimum_target_change_ &&
        y_change < minimum_target_change_ &&
        z_change < minimum_target_change_) {
      return;
    }

    target_publisher_->publish(target);
    last_target_ = target;
  }

  std::string base_frame_;
  std::string tip_link_;

  double workspace_x_scale_;
  double workspace_y_scale_;
  double workspace_z_scale_;
  double maximum_x_offset_;
  int reacquisition_samples_;
  double maximum_y_offset_;
  double maximum_z_offset_;
  double minimum_target_change_;

  double reference_x_;
  double reference_y_;
  double reference_z_;
  double reference_orientation_x_;
  double reference_orientation_y_;
  double reference_orientation_z_;
  double reference_orientation_w_;

  Point latest_palm_center_;
  Point calibrated_palm_center_;
  Point reacquisition_palm_sum_;
  double latest_relative_depth_;
  int reacquisition_sample_count_;

  geometry_msgs::msg::PoseStamped calibrated_pose_;
  geometry_msgs::msg::PoseStamped last_target_;

  bool has_latest_palm_center_;
  bool depth_valid_;
  bool calibrated_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_publisher_;

  rclcpp::Subscription<robot_vision_msgs::msg::HandState>::SharedPtr
      state_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ArmCartesianTeleopNode>());

  rclcpp::shutdown();
  return 0;
}
