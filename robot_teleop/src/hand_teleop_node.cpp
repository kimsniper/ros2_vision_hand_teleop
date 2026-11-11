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
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_vision_msgs/msg/hand_landmarks.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HandTeleopNode : public rclcpp::Node {
public:
  using Point = std::array<double, 3>;
  using HandJoints = std::array<double, 11>;

  HandTeleopNode() : Node("hand_teleop_node"), previous_hand_joints_{} {
    const auto landmarks_topic = declare_parameter<std::string>(
        "landmarks_topic", "/hand/landmarks/filtered");

    const auto command_topic = declare_parameter<std::string>(
        "command_topic", "/hand_position_controller/commands");

    command_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(command_topic, 10);

    landmark_subscription_ =
        create_subscription<robot_vision_msgs::msg::HandLandmarks>(
            landmarks_topic, 10,
            std::bind(&HandTeleopNode::landmarkCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Hand teleop ready: %s -> %s",
                landmarks_topic.c_str(), command_topic.c_str());
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  static constexpr double kFilterPreviousWeight = 0.7;

  static constexpr HandJoints kJointLimits = {
      0.9704,  0.9879,  1.334,   0.79849, 0.79849, 1.334,
      0.98175, 0.98175, 0.28833, 0.28833, 0.5829};

  static Point point(const std::vector<double> &data, std::size_t index) {
    const std::size_t offset = index * 3U;

    return {data[offset], data[offset + 1U], data[offset + 2U]};
  }

  static Point vectorBetween(const Point &start, const Point &end) {
    return {end[0] - start[0], end[1] - start[1], end[2] - start[2]};
  }

  static double dot(const Point &first, const Point &second) {
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
  }

  static double norm(const Point &value) {
    return std::sqrt(dot(value, value));
  }

  static double distance(const Point &first, const Point &second) {
    return norm(vectorBetween(first, second));
  }

  static double angleBetween(const Point &first, const Point &second) {
    const double denominator = norm(first) * norm(second);

    if (denominator <= 1.0e-9) {
      return 0.0;
    }

    const double cosine =
        std::clamp(dot(first, second) / denominator, -1.0, 1.0);

    return std::acos(cosine);
  }

  static double fingerCurl(const Point &mcp, const Point &pip, const Point &dip,
                           const Point &tip) {
    const double proximal_angle =
        angleBetween(vectorBetween(mcp, pip), vectorBetween(pip, dip));

    const double distal_angle =
        angleBetween(vectorBetween(pip, dip), vectorBetween(dip, tip));

    return std::clamp((proximal_angle + distal_angle) / kPi, 0.0, 1.0);
  }

  static double thumbOpposition(const Point &wrist, const Point &index_mcp,
                                const Point &pinky_mcp,
                                const Point &thumb_tip) {
    const double palm_width = distance(index_mcp, pinky_mcp);

    if (palm_width <= 1.0e-9) {
      return 0.0;
    }

    const Point palm_center = {(wrist[0] + index_mcp[0] + pinky_mcp[0]) / 3.0,
                               (wrist[1] + index_mcp[1] + pinky_mcp[1]) / 3.0,
                               (wrist[2] + index_mcp[2] + pinky_mcp[2]) / 3.0};

    const double normalized_distance =
        distance(thumb_tip, palm_center) / palm_width;

    return std::clamp((1.15 - normalized_distance) / 0.75, 0.0, 1.0);
  }

  static double lateralSpread(const Point &middle_mcp, const Point &middle_pip,
                              const Point &finger_mcp, const Point &finger_pip,
                              double full_spread_angle) {
    const Point middle_direction = vectorBetween(middle_mcp, middle_pip);

    const Point finger_direction = vectorBetween(finger_mcp, finger_pip);

    const double spread_angle =
        angleBetween(middle_direction, finger_direction);

    return std::clamp(spread_angle / full_spread_angle, 0.0, 1.0);
  }

  static HandJoints mapHandJoints(const std::vector<double> &data) {
    const Point wrist = point(data, 0U);

    const Point thumb_cmc = point(data, 1U);
    const Point thumb_mcp = point(data, 2U);
    const Point thumb_ip = point(data, 3U);
    const Point thumb_tip = point(data, 4U);

    const Point index_mcp = point(data, 5U);
    const Point index_pip = point(data, 6U);
    const Point index_dip = point(data, 7U);
    const Point index_tip = point(data, 8U);

    const Point middle_mcp = point(data, 9U);
    const Point middle_pip = point(data, 10U);
    const Point middle_dip = point(data, 11U);
    const Point middle_tip = point(data, 12U);

    const Point ring_mcp = point(data, 13U);
    const Point ring_pip = point(data, 14U);
    const Point ring_dip = point(data, 15U);
    const Point ring_tip = point(data, 16U);

    const Point pinky_mcp = point(data, 17U);
    const Point pinky_pip = point(data, 18U);
    const Point pinky_dip = point(data, 19U);
    const Point pinky_tip = point(data, 20U);

    const double thumb_curl =
        fingerCurl(thumb_cmc, thumb_mcp, thumb_ip, thumb_tip);

    const double index_curl =
        fingerCurl(index_mcp, index_pip, index_dip, index_tip);

    const double middle_curl =
        fingerCurl(middle_mcp, middle_pip, middle_dip, middle_tip);

    const double ring_curl = fingerCurl(ring_mcp, ring_pip, ring_dip, ring_tip);

    const double pinky_curl =
        fingerCurl(pinky_mcp, pinky_pip, pinky_dip, pinky_tip);

    constexpr double index_full_spread_angle = 0.20;
    constexpr double ring_full_spread_angle = 0.15;
    constexpr double pinky_full_spread_angle = 0.35;

    const double index_spread = lateralSpread(
        middle_mcp, middle_pip, index_mcp, index_pip, index_full_spread_angle);

    const double ring_spread = lateralSpread(middle_mcp, middle_pip, ring_mcp,
                                             ring_pip, ring_full_spread_angle);

    const double pinky_spread = lateralSpread(
        middle_mcp, middle_pip, pinky_mcp, pinky_pip, pinky_full_spread_angle);

    const HandJoints normalized = {
        thumb_curl,   thumbOpposition(wrist, index_mcp, pinky_mcp, thumb_tip),
        index_curl,   index_curl,
        middle_curl,  middle_curl,
        ring_curl,    pinky_curl,
        index_spread, ring_spread,
        pinky_spread};

    HandJoints mapped{};

    for (std::size_t index = 0U; index < mapped.size(); ++index) {
      mapped[index] = normalized[index] * kJointLimits[index];
    }

    return mapped;
  }

  void landmarkCallback(
      const robot_vision_msgs::msg::HandLandmarks::SharedPtr message) {
    if (!message->hand_detected) {
      return;
    }

    const std::vector<double> hand_data(message->landmarks.begin(),
                                        message->landmarks.end());

    for (const double value : hand_data) {
      if (!std::isfinite(value)) {
        RCLCPP_WARN(get_logger(), "Ignoring non-finite landmark data");
        return;
      }
    }

    const HandJoints mapped = mapHandJoints(hand_data);

    std_msgs::msg::Float64MultiArray command;
    command.data.resize(mapped.size());

    for (std::size_t index = 0U; index < mapped.size(); ++index) {
      const double filtered =
          kFilterPreviousWeight * previous_hand_joints_[index] +
          (1.0 - kFilterPreviousWeight) * mapped[index];

      command.data[index] = filtered;
      previous_hand_joints_[index] = filtered;
    }

    command_publisher_->publish(command);
  }

  HandJoints previous_hand_joints_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      command_publisher_;

  rclcpp::Subscription<robot_vision_msgs::msg::HandLandmarks>::SharedPtr
      landmark_subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<HandTeleopNode>());

  rclcpp::shutdown();
  return 0;
}
