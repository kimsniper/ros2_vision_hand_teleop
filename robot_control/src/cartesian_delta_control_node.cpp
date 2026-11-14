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
#include <memory>
#include <mutex>
#include <string>

#include "robot_control/cartesian_control_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CartesianDeltaControlNode : public CartesianControlNode {
public:
  CartesianDeltaControlNode()
      : CartesianControlNode("cartesian_delta_control_node", false),
        has_latest_cartesian_delta_(false), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    cartesian_delta_topic_ = declare_parameter<std::string>(
        "cartesian_delta_topic", "/arm/cartesian_delta");

    maximum_translation_delta_ =
        declare_parameter<double>("maximum_translation_delta", 0.08);

    maximum_rotation_delta_ =
        declare_parameter<double>("maximum_rotation_delta", 0.20);

    delta_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cartesian_delta_topic_, 10,
        std::bind(&CartesianDeltaControlNode::cartesianDeltaCallback, this,
                  std::placeholders::_1));

    command_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CartesianDeltaControlNode::processLatestCartesianDelta,
                  this));

    RCLCPP_INFO(get_logger(), "Cartesian delta controller ready: %s",
                cartesian_delta_topic_.c_str());
  }

private:
  static double clamp(double value, double magnitude) {
    return std::clamp(value, -magnitude, magnitude);
  }

  static std::array<double, 4>
  quaternionMultiply(const std::array<double, 4> &first,
                     const std::array<double, 4> &second) {
    const double first_x = first[0];
    const double first_y = first[1];
    const double first_z = first[2];
    const double first_w = first[3];

    const double second_x = second[0];
    const double second_y = second[1];
    const double second_z = second[2];
    const double second_w = second[3];

    return {first_w * second_x + first_x * second_w + first_y * second_z -
                first_z * second_y,

            first_w * second_y - first_x * second_z + first_y * second_w +
                first_z * second_x,

            first_w * second_z + first_x * second_y - first_y * second_x +
                first_z * second_w,

            first_w * second_w - first_x * second_x - first_y * second_y -
                first_z * second_z};
  }

  static std::array<double, 4> quaternionFromEuler(double roll, double pitch,
                                                   double yaw) {
    const double half_roll = roll * 0.5;
    const double half_pitch = pitch * 0.5;
    const double half_yaw = yaw * 0.5;

    const double sin_roll = std::sin(half_roll);
    const double cos_roll = std::cos(half_roll);
    const double sin_pitch = std::sin(half_pitch);
    const double cos_pitch = std::cos(half_pitch);
    const double sin_yaw = std::sin(half_yaw);
    const double cos_yaw = std::cos(half_yaw);

    return {sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,

            cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,

            cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,

            cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw};
  }

  static bool normalizeQuaternion(std::array<double, 4> &quaternion) {
    double norm = 0.0;

    for (const double value : quaternion) {
      norm += value * value;
    }

    norm = std::sqrt(norm);

    if (norm <= 1.0e-12) {
      return false;
    }

    for (double &value : quaternion) {
      value /= norm;
    }

    return true;
  }

  static bool isZeroCommand(const geometry_msgs::msg::TwistStamped &message) {
    constexpr double epsilon = 1.0e-9;

    return std::abs(message.twist.linear.x) <= epsilon &&
           std::abs(message.twist.linear.y) <= epsilon &&
           std::abs(message.twist.linear.z) <= epsilon &&
           std::abs(message.twist.angular.x) <= epsilon &&
           std::abs(message.twist.angular.y) <= epsilon &&
           std::abs(message.twist.angular.z) <= epsilon;
  }

  void cartesianDeltaCallback(
      const geometry_msgs::msg::TwistStamped::SharedPtr message) {
    if (!message->header.frame_id.empty() &&
        message->header.frame_id != base_frame_) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring Cartesian delta in frame '%s'; expected '%s'",
                  message->header.frame_id.c_str(), base_frame_.c_str());
      return;
    }

    const std::array<double, 6> values = {
        message->twist.linear.x,  message->twist.linear.y,
        message->twist.linear.z,  message->twist.angular.x,
        message->twist.angular.y, message->twist.angular.z};

    for (const double value : values) {
      if (!std::isfinite(value)) {
        RCLCPP_WARN(get_logger(), "Ignoring non-finite Cartesian delta");
        return;
      }
    }

    std::lock_guard<std::mutex> lock(command_mutex_);

    if (isZeroCommand(*message)) {
      return;
    }

    if (!has_latest_cartesian_delta_) {
      latest_cartesian_delta_ = *message;
      has_latest_cartesian_delta_ = true;
      return;
    }

    latest_cartesian_delta_.header = message->header;

    latest_cartesian_delta_.twist.linear.x =
        clamp(latest_cartesian_delta_.twist.linear.x + message->twist.linear.x,
              maximum_translation_delta_);

    latest_cartesian_delta_.twist.linear.y =
        clamp(latest_cartesian_delta_.twist.linear.y + message->twist.linear.y,
              maximum_translation_delta_);

    latest_cartesian_delta_.twist.linear.z =
        clamp(latest_cartesian_delta_.twist.linear.z + message->twist.linear.z,
              maximum_translation_delta_);

    latest_cartesian_delta_.twist.angular.x = clamp(
        latest_cartesian_delta_.twist.angular.x + message->twist.angular.x,
        maximum_rotation_delta_);

    latest_cartesian_delta_.twist.angular.y = clamp(
        latest_cartesian_delta_.twist.angular.y + message->twist.angular.y,
        maximum_rotation_delta_);

    latest_cartesian_delta_.twist.angular.z = clamp(
        latest_cartesian_delta_.twist.angular.z + message->twist.angular.z,
        maximum_rotation_delta_);
  }

  void processLatestCartesianDelta() {
    if (controlBusy()) {
      return;
    }

    geometry_msgs::msg::TwistStamped command;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);

      if (!has_latest_cartesian_delta_) {
        return;
      }

      command = latest_cartesian_delta_;
      has_latest_cartesian_delta_ = false;
    }

    geometry_msgs::msg::TransformStamped transform;

    try {
      transform =
          tf_buffer_.lookupTransform(base_frame_, tip_link_, tf2::TimePointZero,
                                     tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", error.what());
      return;
    }

    const std::array<double, 3> linear = {
        clamp(command.twist.linear.x, maximum_translation_delta_),
        clamp(command.twist.linear.y, maximum_translation_delta_),
        clamp(command.twist.linear.z, maximum_translation_delta_)};

    const std::array<double, 3> angular = {
        clamp(command.twist.angular.x, maximum_rotation_delta_),
        clamp(command.twist.angular.y, maximum_rotation_delta_),
        clamp(command.twist.angular.z, maximum_rotation_delta_)};

    const std::array<double, 4> current_quaternion = {
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w};

    const auto delta_quaternion =
        quaternionFromEuler(angular[0], angular[1], angular[2]);

    auto target_quaternion =
        quaternionMultiply(current_quaternion, delta_quaternion);

    if (!normalizeQuaternion(target_quaternion)) {
      RCLCPP_WARN(get_logger(), "Could not normalize target quaternion");
      return;
    }

    geometry_msgs::msg::PoseStamped target;

    target.header.stamp = now();
    target.header.frame_id = base_frame_;

    target.pose.position.x = transform.transform.translation.x + linear[0];

    target.pose.position.y = transform.transform.translation.y + linear[1];

    target.pose.position.z = transform.transform.translation.z + linear[2];

    target.pose.orientation.x = target_quaternion[0];

    target.pose.orientation.y = target_quaternion[1];

    target.pose.orientation.z = target_quaternion[2];

    target.pose.orientation.w = target_quaternion[3];

    submitCartesianTarget(target);
  }

  std::string cartesian_delta_topic_;

  double maximum_translation_delta_;
  double maximum_rotation_delta_;

  geometry_msgs::msg::TwistStamped latest_cartesian_delta_;
  bool has_latest_cartesian_delta_;
  std::mutex command_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      delta_subscription_;

  rclcpp::TimerBase::SharedPtr command_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<CartesianDeltaControlNode>();

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
