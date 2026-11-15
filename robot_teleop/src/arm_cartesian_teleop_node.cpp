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
        latest_relative_depth_(0.0), has_latest_palm_center_(false),
        missed_frame_count_(0), depth_valid_(false) {
    const auto state_topic =
        declare_parameter<std::string>("state_topic", "/hand/state");

    const auto target_topic =
        declare_parameter<std::string>("target_topic", "/arm/cartesian_target");

    base_frame_ = declare_parameter<std::string>("base_frame", "link_0");

    tip_link_ = declare_parameter<std::string>("tip_link", "ee_link");

    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);

    workspace_x_forward_scale_ =
        declare_parameter<double>("workspace_x_forward_scale", 0.20);

    workspace_x_backward_scale_ =
        declare_parameter<double>("workspace_x_backward_scale", 0.45);

    enable_depth_control_ =
        declare_parameter<bool>("enable_depth_control", true);

    workspace_y_scale_ = declare_parameter<double>("workspace_y_scale", 0.45);

    workspace_z_scale_ = declare_parameter<double>("workspace_z_scale", 0.60);

    workspace_z_upward_scale_ =
        declare_parameter<double>("workspace_z_upward_scale", 0.12);

    maximum_x_forward_offset_ =
        declare_parameter<double>("maximum_x_forward_offset", 0.04);

    maximum_x_backward_offset_ =
        declare_parameter<double>("maximum_x_backward_offset", 0.14);

    screen_center_x_ = declare_parameter<double>("screen_center_x", 0.50);

    screen_center_y_ = declare_parameter<double>("screen_center_y", 0.50);

    maximum_missed_frames_ =
        declare_parameter<int>("maximum_missed_frames", 10);

    maximum_y_offset_ = declare_parameter<double>("maximum_y_offset", 0.18);

    maximum_z_upward_offset_ =
        declare_parameter<double>("maximum_z_upward_offset", 0.06);

    maximum_z_downward_offset_ =
        declare_parameter<double>("maximum_z_downward_offset", 0.30);

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

    if (workspace_x_forward_scale_ <= 0.0 ||
        workspace_x_backward_scale_ <= 0.0 || workspace_y_scale_ <= 0.0 ||
        workspace_z_scale_ <= 0.0 || workspace_z_upward_scale_ <= 0.0 ||
        maximum_x_forward_offset_ <= 0.0 || maximum_x_backward_offset_ <= 0.0 ||
        maximum_y_offset_ <= 0.0 || maximum_z_upward_offset_ <= 0.0 ||
        maximum_z_downward_offset_ <= 0.0) {
      throw std::invalid_argument("workspace scales and limits "
                                  "must be positive");
    }

    if (maximum_x_forward_offset_ <= 0.0 || maximum_x_backward_offset_ <= 0.0) {
      throw std::invalid_argument("maximum_x_offset must be positive");
    }

    if (!std::isfinite(screen_center_x_) || !std::isfinite(screen_center_y_) ||
        screen_center_x_ < 0.0 || screen_center_x_ > 1.0 ||
        screen_center_y_ < 0.0 || screen_center_y_ > 1.0) {
      throw std::invalid_argument("screen center must be within [0, 1]");
    }

    if (maximum_missed_frames_ < 0) {
      throw std::invalid_argument("maximum_missed_frames must not be negative");
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
  void resetTracking() {
    latest_relative_depth_ = 0.0;
    has_latest_palm_center_ = false;
    missed_frame_count_ = 0;
    depth_valid_ = false;
  }

  void handleTrackingLoss() {
    has_latest_palm_center_ = false;
    depth_valid_ = false;

    ++missed_frame_count_;

    if (missed_frame_count_ > maximum_missed_frames_) {
      resetTracking();
    }
  }

  void
  stateCallback(const robot_vision_msgs::msg::HandState::SharedPtr message) {
    if (!message->hand_detected) {
      handleTrackingLoss();
      return;
    }

    missed_frame_count_ = 0;

    if (!std::isfinite(message->relative_depth) ||
        !std::isfinite(message->palm_center.x) ||
        !std::isfinite(message->palm_center.y) ||
        !std::isfinite(message->palm_center.z) ||
        !std::isfinite(message->palm_scale) || message->palm_scale <= 1.0e-9 ||
        message->landmarks.size() != 63U) {
      resetTracking();
      return;
    }

    latest_palm_center_ = {
        message->palm_center.x,
        message->palm_center.y,
        message->palm_center.z,
    };

    latest_relative_depth_ = message->relative_depth;
    depth_valid_ = message->depth_valid;
    has_latest_palm_center_ = true;
  }

  void publishTarget() {
    if (!has_latest_palm_center_) {
      return;
    }

    geometry_msgs::msg::PoseStamped target = last_target_;

    if (target.header.frame_id.empty()) {
      target.header.frame_id = base_frame_;

      target.pose.position.x = reference_x_;
      target.pose.position.y = reference_y_;
      target.pose.position.z = reference_z_;

      target.pose.orientation.x = reference_orientation_x_;
      target.pose.orientation.y = reference_orientation_y_;
      target.pose.orientation.z = reference_orientation_z_;
      target.pose.orientation.w = reference_orientation_w_;
    }

    target.header.stamp = now();

    const double palm_offset_x = latest_palm_center_[0] - screen_center_x_;

    const double palm_offset_y = latest_palm_center_[1] - screen_center_y_;

    const double x_offset =
        enable_depth_control_ && depth_valid_
            ? (latest_relative_depth_ >= 0.0
                   ? std::clamp(latest_relative_depth_ *
                                    workspace_x_forward_scale_,
                                0.0, maximum_x_forward_offset_)
                   : -std::clamp(-latest_relative_depth_ *
                                     workspace_x_backward_scale_,
                                 0.0, maximum_x_backward_offset_))
            : 0.0;

    const double y_offset = std::clamp(-palm_offset_x * workspace_y_scale_,
                                       -maximum_y_offset_, maximum_y_offset_);

    const double z_offset =
        palm_offset_y >= 0.0
            ? -std::clamp(palm_offset_y * workspace_z_scale_, 0.0,
                          maximum_z_downward_offset_)
            : std::clamp(-palm_offset_y * workspace_z_upward_scale_, 0.0,
                         maximum_z_upward_offset_);

    target.pose.position.x = reference_x_ + x_offset;

    target.pose.position.y = reference_y_ + y_offset;

    target.pose.position.z = reference_z_ + z_offset;

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

  double workspace_x_forward_scale_;
  double workspace_x_backward_scale_;
  bool enable_depth_control_;
  double workspace_y_scale_;
  double workspace_z_scale_;
  double workspace_z_upward_scale_;
  double maximum_x_forward_offset_;
  double maximum_x_backward_offset_;
  double screen_center_x_;
  double screen_center_y_;
  int maximum_missed_frames_;
  double maximum_y_offset_;
  double maximum_z_upward_offset_;
  double maximum_z_downward_offset_;
  double minimum_target_change_;
  double reference_x_;
  double reference_y_;
  double reference_z_;
  double reference_orientation_x_;
  double reference_orientation_y_;
  double reference_orientation_z_;
  double reference_orientation_w_;

  Point latest_palm_center_;
  double latest_relative_depth_;

  geometry_msgs::msg::PoseStamped last_target_;

  bool has_latest_palm_center_;
  int missed_frame_count_;
  bool depth_valid_;

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
