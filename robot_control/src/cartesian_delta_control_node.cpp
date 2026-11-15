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

  static bool isZeroCommand(const geometry_msgs::msg::TwistStamped &message) {
    constexpr double epsilon = 1.0e-9;

    return std::abs(message.twist.linear.x) <= epsilon &&
           std::abs(message.twist.linear.y) <= epsilon &&
           std::abs(message.twist.linear.z) <= epsilon;
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

    const std::array<double, 3> values = {message->twist.linear.x,
                                          message->twist.linear.y,
                                          message->twist.linear.z};

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

    geometry_msgs::msg::PoseStamped target;

    target.header.stamp = now();
    target.header.frame_id = base_frame_;

    target.pose.position.x = transform.transform.translation.x + linear[0];

    target.pose.position.y = transform.transform.translation.y + linear[1];

    target.pose.position.z = transform.transform.translation.z + linear[2];

    target.pose.orientation = transform.transform.rotation;

    submitCartesianTarget(target);
  }

  std::string cartesian_delta_topic_;

  double maximum_translation_delta_;
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
