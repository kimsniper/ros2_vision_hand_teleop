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
#include "robot_state_estimation/palm_state_extractor.hpp"
#include "robot_vision_msgs/msg/hand_landmarks.hpp"
#include "robot_vision_msgs/msg/hand_state.hpp"

namespace robot_state_estimation {

class StateEstimatorNode : public rclcpp::Node {
public:
  StateEstimatorNode() : Node("state_estimator_node") {
    const std::string input_topic =
        declare_parameter<std::string>("input_topic", "/hand/landmarks");

    const std::string output_topic = declare_parameter<std::string>(
        "output_topic", "/hand/landmarks/filtered");

    const std::string state_topic =
        declare_parameter<std::string>("state_topic", "/hand/state");

    process_noise_ = declarePositiveParameter("process_noise", 0.1);

    measurement_noise_ = declarePositiveParameter("measurement_noise", 0.01);

    initial_position_variance_ =
        declarePositiveParameter("initial_position_variance", 1.0);

    initial_velocity_variance_ =
        declarePositiveParameter("initial_velocity_variance", 1.0);

    minimum_dt_ = declarePositiveParameter("minimum_dt", 0.001);

    maximum_dt_ = declarePositiveParameter("maximum_dt", 0.5);

    depth_scale_deadband_ =
        declarePositiveParameter("depth_scale_deadband", 0.03);

    minimum_palm_normal_z_ =
        declare_parameter<double>("minimum_palm_normal_z", 0.70);

    if (!std::isfinite(minimum_palm_normal_z_) ||
        minimum_palm_normal_z_ < 0.0 || minimum_palm_normal_z_ > 1.0) {
      throw std::invalid_argument(
          "minimum_palm_normal_z must be within [0, 1]");
    }

    if (minimum_dt_ > maximum_dt_) {
      throw std::invalid_argument("minimum_dt must not exceed maximum_dt");
    }

    filters_.reserve(kLandmarkValueCount);

    for (std::size_t index = 0; index < kLandmarkValueCount; ++index) {
      filters_.emplace_back(process_noise_, measurement_noise_,
                            initial_position_variance_,
                            initial_velocity_variance_);
    }

    depth_filter_ = std::make_unique<ConstantVelocityKalmanFilter>(
        process_noise_, measurement_noise_, initial_position_variance_,
        initial_velocity_variance_);

    publisher_ = create_publisher<robot_vision_msgs::msg::HandLandmarks>(
        output_topic, 10);

    state_publisher_ =
        create_publisher<robot_vision_msgs::msg::HandState>(state_topic, 10);

    subscription_ = create_subscription<robot_vision_msgs::msg::HandLandmarks>(
        input_topic, 10,
        std::bind(&StateEstimatorNode::landmarkCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Filtering %s -> %s and %s", input_topic.c_str(),
                output_topic.c_str(), state_topic.c_str());
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

  void resetDepthState() {
    depth_filter_->reset();
    reference_palm_scale_ = 0.0;
    depth_reference_initialized_ = false;
  }

  void resetFilters() {
    for (auto &filter : filters_) {
      filter.reset();
    }

    resetDepthState();
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

    robot_vision_msgs::msg::HandState state;

    state.header = input.header;
    state.landmarks.fill(0.0);
    state.hand_detected = false;
    state.depth_valid = false;

    publisher_->publish(output);
    state_publisher_->publish(state);
  }

  void publishState(const robot_vision_msgs::msg::HandLandmarks &landmarks,
                    double dt, bool reinitialize_depth) {
    robot_vision_msgs::msg::HandState state;

    state.header = landmarks.header;
    state.landmarks = landmarks.landmarks;
    state.hand_detected = landmarks.hand_detected;

    const PalmFeatures palm = PalmStateExtractor::extract(
        landmarks.landmarks, minimum_palm_normal_z_);

    state.palm_center.x = palm.center[0];
    state.palm_center.y = palm.center[1];
    state.palm_center.z = palm.center[2];

    state.palm_normal.x = palm.normal[0];
    state.palm_normal.y = palm.normal[1];
    state.palm_normal.z = palm.normal[2];

    state.palm_scale = palm.scale;

    if (!palm.depth_valid) {
      resetDepthState();
      state.relative_depth = 0.0;
      state.depth_velocity = 0.0;
      state.depth_valid = false;
      state_publisher_->publish(state);
      return;
    }

    if (reinitialize_depth || !depth_reference_initialized_) {
      resetDepthState();
      reference_palm_scale_ = palm.scale;
      depth_reference_initialized_ = true;
      depth_filter_->initialize(0.0);

      state.relative_depth = 0.0;
      state.depth_velocity = 0.0;
      state.depth_valid = true;
      state_publisher_->publish(state);
      return;
    }

    const double scale_ratio = palm.scale / reference_palm_scale_;
    const double scale_change = std::abs(scale_ratio - 1.0);
    const double measured_depth =
        scale_change <= depth_scale_deadband_ ? 0.0 : std::log(scale_ratio);

    state.relative_depth = depth_filter_->update(measured_depth, dt);
    state.depth_velocity = depth_filter_->velocity();
    state.depth_valid = true;

    state_publisher_->publish(state);
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
      publishState(output, minimum_dt_, true);
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
      publishState(output, minimum_dt_, true);
      return;
    }

    const double dt = std::max(elapsed_seconds, minimum_dt_);

    for (std::size_t index = 0; index < kLandmarkValueCount; ++index) {
      output.landmarks[index] =
          filters_[index].update(message->landmarks[index], dt);
    }

    previous_timestamp_nanoseconds_ = timestamp_nanoseconds;

    publisher_->publish(output);
    publishState(output, dt, false);
  }

  double process_noise_;
  double measurement_noise_;
  double initial_position_variance_;
  double initial_velocity_variance_;
  double minimum_dt_;
  double maximum_dt_;
  double depth_scale_deadband_;
  double minimum_palm_normal_z_;

  std::vector<ConstantVelocityKalmanFilter> filters_;
  std::unique_ptr<ConstantVelocityKalmanFilter> depth_filter_;

  double reference_palm_scale_{0.0};
  bool depth_reference_initialized_{false};

  std::int64_t previous_timestamp_nanoseconds_{0};
  bool timestamp_initialized_{false};

  rclcpp::Publisher<robot_vision_msgs::msg::HandLandmarks>::SharedPtr
      publisher_;

  rclcpp::Publisher<robot_vision_msgs::msg::HandState>::SharedPtr
      state_publisher_;

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
