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
#include "robot_vision_msgs/msg/hand_state.hpp"

class ArmCartesianDeltaTeleopNode : public rclcpp::Node {
public:
  using Point = std::array<double, 3>;

  ArmCartesianDeltaTeleopNode()
      : Node("arm_cartesian_delta_teleop_node"), previous_palm_center_{0.0, 0.0,
                                                                       0.0},
        latest_depth_velocity_(0.0), has_previous_palm_center_(false),
        depth_mode_active_(false), depth_valid_(false) {
    const auto state_topic =
        declare_parameter<std::string>("state_topic", "/hand/state");

    const auto cartesian_delta_topic = declare_parameter<std::string>(
        "cartesian_delta_topic", "/arm/cartesian_delta");

    lateral_scale_ = declare_parameter<double>("lateral_scale", 1.50);

    vertical_scale_ = declare_parameter<double>("vertical_scale", 1.50);

    sample_deadzone_ = declare_parameter<double>("sample_deadzone", 0.002140);

    depth_mode_enter_threshold_ =
        declare_parameter<double>("depth_mode_enter_threshold", 0.014039);

    depth_mode_exit_threshold_ =
        declare_parameter<double>("depth_mode_exit_threshold", 0.010932);

    depth_mode_planar_deadzone_ =
        declare_parameter<double>("depth_mode_planar_deadzone", 0.006000);

    depth_forward_scale_ =
        declare_parameter<double>("depth_forward_scale", 0.10);

    depth_backward_scale_ =
        declare_parameter<double>("depth_backward_scale", 0.225);

    maximum_depth_forward_delta_ =
        declare_parameter<double>("maximum_depth_forward_delta", 0.003);

    maximum_depth_backward_delta_ =
        declare_parameter<double>("maximum_depth_backward_delta", 0.007);

    enable_depth_control_ =
        declare_parameter<bool>("enable_depth_control", true);

    screen_center_x_ = declare_parameter<double>("screen_center_x", 0.50);

    screen_center_y_ = declare_parameter<double>("screen_center_y", 0.50);

    screen_deadzone_ = declare_parameter<double>("screen_deadzone", 0.02);

    maximum_cartesian_delta_ =
        declare_parameter<double>("max_cartesian_delta_m", 0.01);

    if (depth_forward_scale_ <= 0.0 || depth_backward_scale_ <= 0.0 ||
        maximum_depth_forward_delta_ <= 0.0 ||
        maximum_depth_backward_delta_ <= 0.0) {
      throw std::invalid_argument("depth scales and limits must be positive");
    }

    if (lateral_scale_ <= 0.0 || vertical_scale_ <= 0.0 ||
        sample_deadzone_ < 0.0) {
      throw std::invalid_argument("sample displacement parameters are invalid");
    }

    if (depth_mode_enter_threshold_ <= 0.0 ||
        depth_mode_exit_threshold_ <= 0.0 ||
        depth_mode_exit_threshold_ >= depth_mode_enter_threshold_ ||
        depth_mode_planar_deadzone_ < sample_deadzone_) {
      throw std::invalid_argument(
          "depth-mode hysteresis parameters are invalid");
    }

    cartesian_delta_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(
            cartesian_delta_topic, 10);

    state_subscription_ =
        create_subscription<robot_vision_msgs::msg::HandState>(
            state_topic, 10,
            std::bind(&ArmCartesianDeltaTeleopNode::stateCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "3D screen-centered Cartesian delta arm teleop ready: "
                "%s -> %s",
                state_topic.c_str(), cartesian_delta_topic.c_str());
  }

private:
  void resetTracking() {
    latest_depth_velocity_ = 0.0;
    has_previous_palm_center_ = false;
    depth_mode_active_ = false;
    depth_valid_ = false;
    publishStop();
  }

  void
  stateCallback(const robot_vision_msgs::msg::HandState::SharedPtr message) {
    if (!message->hand_detected) {
      resetTracking();
      return;
    }

    if (!std::isfinite(message->palm_center.x) ||
        !std::isfinite(message->palm_center.y) ||
        !std::isfinite(message->palm_center.z) ||
        !std::isfinite(message->depth_velocity)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite hand state");

      resetTracking();
      return;
    }

    const Point current_palm_center = {
        message->palm_center.x, message->palm_center.y, message->palm_center.z};

    latest_depth_velocity_ = message->depth_velocity;

    depth_valid_ = message->depth_valid;

    if (!has_previous_palm_center_) {
      previous_palm_center_ = current_palm_center;

      has_previous_palm_center_ = true;
      publishStop();
      return;
    }

    const double raw_hand_delta_x =
        current_palm_center[0] - previous_palm_center_[0];

    const double raw_hand_delta_y =
        current_palm_center[1] - previous_palm_center_[1];

    previous_palm_center_ = current_palm_center;

    const double planar_magnitude =
        std::max(std::abs(raw_hand_delta_x), std::abs(raw_hand_delta_y));

    const bool planar_override =
        planar_magnitude >= depth_mode_planar_deadzone_;

    const double depth_speed =
        depth_valid_ ? std::abs(latest_depth_velocity_) : 0.0;

    if (planar_override) {
      depth_mode_active_ = false;
    } else if (depth_mode_active_) {
      if (depth_speed <= depth_mode_exit_threshold_) {
        depth_mode_active_ = false;
      }
    } else if (depth_speed >= depth_mode_enter_threshold_) {
      depth_mode_active_ = true;
    }

    double delta_x = 0.0;
    double delta_y = 0.0;
    double delta_z = 0.0;

    if (planar_override || !depth_mode_active_) {
      const double hand_delta_x =
          applyDeadzone(raw_hand_delta_x, sample_deadzone_);

      const double hand_delta_y =
          applyDeadzone(raw_hand_delta_y, sample_deadzone_);

      delta_y = std::clamp(-hand_delta_x * lateral_scale_,
                           -maximum_cartesian_delta_, maximum_cartesian_delta_);

      delta_z = std::clamp(-hand_delta_y * vertical_scale_,
                           -maximum_cartesian_delta_, maximum_cartesian_delta_);
    } else if (enable_depth_control_ && depth_valid_) {
      delta_x =
          latest_depth_velocity_ >= 0.0
              ? std::clamp(latest_depth_velocity_ * depth_forward_scale_, 0.0,
                           maximum_depth_forward_delta_)
              : -std::clamp(-latest_depth_velocity_ * depth_backward_scale_,
                            0.0, maximum_depth_backward_delta_);
    }

    publishCartesianDelta(delta_x, delta_y, delta_z);
  }

  static double applyDeadzone(double value, double deadzone) {
    if (std::abs(value) <= deadzone) {
      return 0.0;
    }

    const double sign = value < 0.0 ? -1.0 : 1.0;

    return sign * (std::abs(value) - deadzone) / (0.5 - deadzone);
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

  void publishCartesianDelta(double delta_x, double delta_y, double delta_z) {
    geometry_msgs::msg::TwistStamped command;

    command.header.stamp = now();
    command.header.frame_id = "link_0";

    command.twist.linear.x = delta_x;
    command.twist.linear.y = delta_y;
    command.twist.linear.z = delta_z;

    command.twist.angular.x = 0.0;
    command.twist.angular.y = 0.0;
    command.twist.angular.z = 0.0;

    cartesian_delta_publisher_->publish(command);
  }

  double lateral_scale_;
  double vertical_scale_;
  double sample_deadzone_;
  double depth_mode_enter_threshold_;
  double depth_mode_exit_threshold_;
  double depth_mode_planar_deadzone_;

  double depth_forward_scale_;
  double depth_backward_scale_;
  double maximum_depth_forward_delta_;
  double maximum_depth_backward_delta_;
  bool enable_depth_control_;
  double screen_center_x_;
  double screen_center_y_;
  double screen_deadzone_;
  double maximum_cartesian_delta_;

  Point previous_palm_center_;
  double latest_depth_velocity_;

  bool has_previous_palm_center_;
  bool depth_mode_active_;
  bool depth_valid_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cartesian_delta_publisher_;

  rclcpp::Subscription<robot_vision_msgs::msg::HandState>::SharedPtr
      state_subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ArmCartesianDeltaTeleopNode>());

  rclcpp::shutdown();
  return 0;
}
