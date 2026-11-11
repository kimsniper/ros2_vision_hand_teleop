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
#include "robot_vision_msgs/msg/hand_landmarks.hpp"

class ArmCartesianTeleopNode : public rclcpp::Node {
public:
  using Point = std::array<double, 3>;

  ArmCartesianTeleopNode()
      : Node("arm_cartesian_teleop_node"), latest_palm_center_{0.0, 0.0, 0.0},
        calibrated_palm_center_{0.0, 0.0, 0.0}, has_latest_palm_center_(false),
        calibrated_(false) {
    const auto landmarks_topic =
        declare_parameter<std::string>("landmarks_topic", "/hand/landmarks");

    const auto target_topic =
        declare_parameter<std::string>("target_topic", "/arm/cartesian_target");

    base_frame_ = declare_parameter<std::string>("base_frame", "link_0");

    tip_link_ = declare_parameter<std::string>("tip_link", "ee_link");

    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);

    workspace_y_scale_ = declare_parameter<double>("workspace_y_scale", 0.60);

    workspace_z_scale_ = declare_parameter<double>("workspace_z_scale", 0.50);

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

    target_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, 10);

    landmark_subscription_ =
        create_subscription<robot_vision_msgs::msg::HandLandmarks>(
            landmarks_topic, 10,
            std::bind(&ArmCartesianTeleopNode::landmarkCallback, this,
                      std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz),
        std::bind(&ArmCartesianTeleopNode::publishTarget, this));

    RCLCPP_INFO(get_logger(), "Cartesian arm teleop ready: %s -> %s",
                landmarks_topic.c_str(), target_topic.c_str());
  }

private:
  static Point getPoint(const std::vector<double> &data, std::size_t index) {
    const std::size_t offset = index * 3U;

    return {data[offset], data[offset + 1U], data[offset + 2U]};
  }

  static Point palmCenter(const std::vector<double> &data) {
    static constexpr std::array<std::size_t, 5> palm_indices = {0U, 5U, 9U, 13U,
                                                                17U};

    Point center{0.0, 0.0, 0.0};

    for (const std::size_t index : palm_indices) {
      const Point landmark = getPoint(data, index);

      center[0] += landmark[0];
      center[1] += landmark[1];
      center[2] += landmark[2];
    }

    constexpr double landmark_count = 5.0;

    center[0] /= landmark_count;
    center[1] /= landmark_count;
    center[2] /= landmark_count;

    return center;
  }

  void resetTracking() {
    has_latest_palm_center_ = false;
    calibrated_ = false;
  }

  void landmarkCallback(
      const robot_vision_msgs::msg::HandLandmarks::SharedPtr message) {
    if (!message->hand_detected) {
      resetTracking();
      return;
    }

    const std::vector<double> hand_data(message->landmarks.begin(),
                                        message->landmarks.end());

    for (const double value : hand_data) {
      if (!std::isfinite(value)) {
        RCLCPP_WARN(get_logger(), "Ignoring non-finite landmark data");
        resetTracking();
        return;
      }
    }

    latest_palm_center_ = palmCenter(hand_data);

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

    RCLCPP_INFO(get_logger(),
                "Cartesian arm teleop calibrated at "
                "palm=(%.3f, %.3f), "
                "reference=(%.3f, %.3f, %.3f)",
                calibrated_palm_center_[0], calibrated_palm_center_[1],
                reference_x_, reference_y_, reference_z_);

    return true;
  }

  void publishTarget() {
    if (!has_latest_palm_center_) {
      return;
    }

    if (!calibrated_ && !calibrate()) {
      return;
    }

    const double palm_offset_x =
        latest_palm_center_[0] - calibrated_palm_center_[0];

    const double palm_offset_y =
        latest_palm_center_[1] - calibrated_palm_center_[1];

    const double y_offset = std::clamp(-palm_offset_x * workspace_y_scale_,
                                       -maximum_y_offset_, maximum_y_offset_);

    const double z_offset = std::clamp(-palm_offset_y * workspace_z_scale_,
                                       -maximum_z_offset_, maximum_z_offset_);

    geometry_msgs::msg::PoseStamped target = calibrated_pose_;

    target.header.stamp = now();

    target.pose.position.y = calibrated_pose_.pose.position.y + y_offset;

    target.pose.position.z = calibrated_pose_.pose.position.z + z_offset;

    const double y_change =
        std::abs(target.pose.position.y - last_target_.pose.position.y);

    const double z_change =
        std::abs(target.pose.position.z - last_target_.pose.position.z);

    if (y_change < minimum_target_change_ &&
        z_change < minimum_target_change_) {
      return;
    }

    target_publisher_->publish(target);
    last_target_ = target;
  }

  std::string base_frame_;
  std::string tip_link_;

  double workspace_y_scale_;
  double workspace_z_scale_;
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

  geometry_msgs::msg::PoseStamped calibrated_pose_;
  geometry_msgs::msg::PoseStamped last_target_;

  bool has_latest_palm_center_;
  bool calibrated_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_publisher_;

  rclcpp::Subscription<robot_vision_msgs::msg::HandLandmarks>::SharedPtr
      landmark_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ArmCartesianTeleopNode>());

  rclcpp::shutdown();
  return 0;
}
