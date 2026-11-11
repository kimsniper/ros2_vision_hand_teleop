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
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_state_estimation/constant_velocity_kalman_filter.hpp"
#include "robot_vision_msgs/msg/hand_landmarks.hpp"

namespace robot_state_estimation {

class StateEstimatorNode : public rclcpp::Node {
public:
  StateEstimatorNode() : Node("state_estimator_node") {
    const std::string input_topic =
        declare_parameter<std::string>("input_topic", "/hand/landmarks");

    const std::string output_topic = declare_parameter<std::string>(
        "output_topic", "/hand/landmarks/filtered");

    process_noise_ = declarePositiveParameter("process_noise", 0.1);

    measurement_noise_ = declarePositiveParameter("measurement_noise", 0.01);

    initial_position_variance_ =
        declarePositiveParameter("initial_position_variance", 1.0);

    initial_velocity_variance_ =
        declarePositiveParameter("initial_velocity_variance", 1.0);

    minimum_dt_ = declarePositiveParameter("minimum_dt", 0.001);

    maximum_dt_ = declarePositiveParameter("maximum_dt", 0.5);

    if (minimum_dt_ > maximum_dt_) {
      throw std::invalid_argument("minimum_dt must not exceed maximum_dt");
    }

    filters_.reserve(kLandmarkValueCount);

    for (std::size_t index = 0; index < kLandmarkValueCount; ++index) {
      filters_.emplace_back(process_noise_, measurement_noise_,
                            initial_position_variance_,
                            initial_velocity_variance_);
    }

    publisher_ = create_publisher<robot_vision_msgs::msg::HandLandmarks>(
        output_topic, 10);

    subscription_ = create_subscription<robot_vision_msgs::msg::HandLandmarks>(
        input_topic, 10,
        std::bind(&StateEstimatorNode::landmarkCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Filtering %s -> %s", input_topic.c_str(),
                output_topic.c_str());
  }

private:
  static constexpr std::size_t kLandmarkValueCount = 63U;
  static constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

  double declarePositiveParameter(const std::string &name,
                                  double default_value) {
    const double value = declare_parameter<double>(name, default_value);

    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }

    return value;
  }

  static std::int64_t
  timestampNanoseconds(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  void resetFilters() {
    for (auto &filter : filters_) {
      filter.reset();
    }

    previous_timestamp_nanoseconds_ = 0;
    timestamp_initialized_ = false;
  }

  void
  initializeFilters(const std::array<double, kLandmarkValueCount> &landmarks) {
    for (std::size_t index = 0; index < landmarks.size(); ++index) {
      filters_[index].initialize(landmarks[index]);
    }
  }

  bool landmarksAreFinite(
      const std::array<double, kLandmarkValueCount> &landmarks) const {
    return std::all_of(landmarks.begin(), landmarks.end(),
                       [](double value) { return std::isfinite(value); });
  }

  void publishUndetected(const robot_vision_msgs::msg::HandLandmarks &input) {
    robot_vision_msgs::msg::HandLandmarks output;

    output.header = input.header;
    output.landmarks.fill(0.0);
    output.hand_detected = false;

    publisher_->publish(output);
  }

  void landmarkCallback(
      const robot_vision_msgs::msg::HandLandmarks::SharedPtr message) {
    if (!message->hand_detected) {
      resetFilters();
      publishUndetected(*message);
      return;
    }

    if (!landmarksAreFinite(message->landmarks)) {
      RCLCPP_WARN(get_logger(),
                  "Resetting estimator after non-finite landmark input");

      resetFilters();
      publishUndetected(*message);
      return;
    }

    const std::int64_t timestamp_nanoseconds =
        timestampNanoseconds(message->header.stamp);

    robot_vision_msgs::msg::HandLandmarks output;

    output.header = message->header;
    output.hand_detected = true;

    if (!timestamp_initialized_ || timestamp_nanoseconds <= 0) {
      initializeFilters(message->landmarks);
      output.landmarks = message->landmarks;

      previous_timestamp_nanoseconds_ = timestamp_nanoseconds;
      timestamp_initialized_ = timestamp_nanoseconds > 0;

      publisher_->publish(output);
      return;
    }

    const std::int64_t elapsed_nanoseconds =
        timestamp_nanoseconds - previous_timestamp_nanoseconds_;

    const double elapsed_seconds = static_cast<double>(elapsed_nanoseconds) /
                                   static_cast<double>(kNanosecondsPerSecond);

    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0 ||
        elapsed_seconds > maximum_dt_) {
      initializeFilters(message->landmarks);
      output.landmarks = message->landmarks;

      previous_timestamp_nanoseconds_ = timestamp_nanoseconds;

      publisher_->publish(output);
      return;
    }

    const double dt = std::max(elapsed_seconds, minimum_dt_);

    for (std::size_t index = 0; index < kLandmarkValueCount; ++index) {
      output.landmarks[index] =
          filters_[index].update(message->landmarks[index], dt);
    }

    previous_timestamp_nanoseconds_ = timestamp_nanoseconds;

    publisher_->publish(output);
  }

  double process_noise_;
  double measurement_noise_;
  double initial_position_variance_;
  double initial_velocity_variance_;
  double minimum_dt_;
  double maximum_dt_;

  std::vector<ConstantVelocityKalmanFilter> filters_;

  std::int64_t previous_timestamp_nanoseconds_{0};
  bool timestamp_initialized_{false};

  rclcpp::Publisher<robot_vision_msgs::msg::HandLandmarks>::SharedPtr
      publisher_;

  rclcpp::Subscription<robot_vision_msgs::msg::HandLandmarks>::SharedPtr
      subscription_;
};

} // namespace robot_state_estimation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<robot_state_estimation::StateEstimatorNode>());

  rclcpp::shutdown();

  return 0;
}
