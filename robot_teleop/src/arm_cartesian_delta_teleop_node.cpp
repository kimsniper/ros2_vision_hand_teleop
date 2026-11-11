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

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_vision_msgs/msg/hand_landmarks.hpp"

class ArmCartesianDeltaTeleopNode : public rclcpp::Node {
public:
  using Point = std::array<double, 3>;

  ArmCartesianDeltaTeleopNode()
      : Node("arm_cartesian_delta_teleop_node"),
        latest_palm_center_{0.0, 0.0, 0.0}, filtered_cartesian_delta_{0.0, 0.0,
                                                                      0.0},
        has_latest_palm_center_(false) {
    const auto landmarks_topic =
        declare_parameter<std::string>("landmarks_topic", "/hand/landmarks");

    const auto cartesian_delta_topic = declare_parameter<std::string>(
        "cartesian_delta_topic", "/arm/cartesian_delta");

    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 15.0);

    lateral_gain_ = declare_parameter<double>("lateral_gain", 0.08);

    vertical_gain_ = declare_parameter<double>("vertical_gain", 0.08);

    screen_center_x_ = declare_parameter<double>("screen_center_x", 0.50);

    screen_center_y_ = declare_parameter<double>("screen_center_y", 0.50);

    screen_deadzone_ = declare_parameter<double>("screen_deadzone", 0.003);

    maximum_cartesian_delta_ =
        declare_parameter<double>("max_cartesian_delta_m", 0.02);

    filter_alpha_ = declare_parameter<double>("alpha", 0.10);

    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    cartesian_delta_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(
            cartesian_delta_topic, 10);

    landmark_subscription_ =
        create_subscription<robot_vision_msgs::msg::HandLandmarks>(
            landmarks_topic, 10,
            std::bind(&ArmCartesianDeltaTeleopNode::landmarkCallback, this,
                      std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz),
        std::bind(&ArmCartesianDeltaTeleopNode::publishCartesianDelta, this));

    RCLCPP_INFO(get_logger(),
                "Screen-centered Cartesian delta arm teleop ready: %s -> %s",
                landmarks_topic.c_str(), cartesian_delta_topic.c_str());
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

    constexpr double palm_landmark_count =
        static_cast<double>(palm_indices.size());

    center[0] /= palm_landmark_count;
    center[1] /= palm_landmark_count;
    center[2] /= palm_landmark_count;

    return center;
  }

  void resetTracking() {
    has_latest_palm_center_ = false;
    filtered_cartesian_delta_ = {0.0, 0.0, 0.0};
    publishStop();
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

  static double applyDeadzone(double value, double deadzone) {
    if (std::abs(value) <= deadzone) {
      return 0.0;
    }

    const double sign = value < 0.0 ? -1.0 : 1.0;

    return sign * (std::abs(value) - deadzone) / (0.5 - deadzone);
  }

  double filterValue(std::size_t index, double value) {
    value =
        std::clamp(value, -maximum_cartesian_delta_, maximum_cartesian_delta_);

    double filtered = filter_alpha_ * filtered_cartesian_delta_[index] +
                      (1.0 - filter_alpha_) * value;

    if (value == 0.0 && std::abs(filtered) < 0.0005) {
      filtered = 0.0;
    }

    filtered_cartesian_delta_[index] = filtered;
    return filtered;
  }

  void publishStop() {
    geometry_msgs::msg::TwistStamped command;

    command.header.stamp = now();
    command.header.frame_id = "link_0";

    command.twist.linear.x = 0.0;
    command.twist.linear.y = 0.0;
    command.twist.linear.z = 0.0;

    command.twist.angular.x = 0.0;
    command.twist.angular.y = 0.0;
    command.twist.angular.z = 0.0;

    cartesian_delta_publisher_->publish(command);
  }

  void publishCartesianDelta() {
    if (!has_latest_palm_center_) {
      publishStop();
      return;
    }

    const double screen_error_x = latest_palm_center_[0] - screen_center_x_;

    const double screen_error_y = latest_palm_center_[1] - screen_center_y_;

    const double normalized_x = applyDeadzone(screen_error_x, screen_deadzone_);

    const double normalized_y = applyDeadzone(screen_error_y, screen_deadzone_);

    const double delta_y = filterValue(1U, -normalized_x * lateral_gain_);

    const double delta_z = filterValue(2U, -normalized_y * vertical_gain_);

    geometry_msgs::msg::TwistStamped command;

    command.header.stamp = now();
    command.header.frame_id = "link_0";

    command.twist.linear.x = 0.0;
    command.twist.linear.y = delta_y;
    command.twist.linear.z = delta_z;

    command.twist.angular.x = 0.0;
    command.twist.angular.y = 0.0;
    command.twist.angular.z = 0.0;

    cartesian_delta_publisher_->publish(command);
  }

  double lateral_gain_;
  double vertical_gain_;
  double screen_center_x_;
  double screen_center_y_;
  double screen_deadzone_;
  double maximum_cartesian_delta_;
  double filter_alpha_;

  Point latest_palm_center_;
  Point filtered_cartesian_delta_;

  bool has_latest_palm_center_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cartesian_delta_publisher_;

  rclcpp::Subscription<robot_vision_msgs::msg::HandLandmarks>::SharedPtr
      landmark_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ArmCartesianDeltaTeleopNode>());

  rclcpp::shutdown();
  return 0;
}
