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
  using Quaternion = std::array<double, 4>;

  ArmCartesianTeleopNode()
      : Node("arm_cartesian_teleop_node"), latest_palm_center_{0.0, 0.0, 0.0},
        latest_palm_orientation_{0.0, 0.0, 0.0, 1.0},
        reference_palm_orientation_{0.0, 0.0, 0.0, 1.0},
        latest_relative_depth_(0.0), has_latest_palm_center_(false),
        has_latest_palm_orientation_(false),
        orientation_reference_initialized_(false), missed_frame_count_(0),
        depth_valid_(false) {
    const auto state_topic =
        declare_parameter<std::string>("state_topic", "/hand/state");

    const auto target_topic =
        declare_parameter<std::string>("target_topic", "/arm/cartesian_target");

    base_frame_ = declare_parameter<std::string>("base_frame", "link_0");

    tip_link_ = declare_parameter<std::string>("tip_link", "ee_link");

    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);

    workspace_x_scale_ = declare_parameter<double>("workspace_x_scale", 0.15);

    enable_depth_control_ =
        declare_parameter<bool>("enable_depth_control", false);

    workspace_y_scale_ = declare_parameter<double>("workspace_y_scale", 0.45);

    workspace_z_scale_ = declare_parameter<double>("workspace_z_scale", 0.60);

    workspace_z_upward_scale_ =
        declare_parameter<double>("workspace_z_upward_scale", 0.12);

    maximum_x_offset_ = declare_parameter<double>("maximum_x_offset", 0.08);

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

    maximum_orientation_offset_ =
        declare_parameter<double>("maximum_orientation_offset", 0.174533);

    orientation_deadband_ =
        declare_parameter<double>("orientation_deadband", 0.0261799);

    minimum_orientation_change_ =
        declare_parameter<double>("minimum_orientation_change", 0.0174533);

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

    if (workspace_x_scale_ <= 0.0 || workspace_y_scale_ <= 0.0 ||
        workspace_z_scale_ <= 0.0 || workspace_z_upward_scale_ <= 0.0 ||
        maximum_x_offset_ <= 0.0 || maximum_y_offset_ <= 0.0 ||
        maximum_z_upward_offset_ <= 0.0 || maximum_z_downward_offset_ <= 0.0) {
      throw std::invalid_argument("workspace scales and limits "
                                  "must be positive");
    }

    if (maximum_x_offset_ <= 0.0) {
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

    if (maximum_orientation_offset_ <= 0.0 || orientation_deadband_ < 0.0 ||
        minimum_orientation_change_ <= 0.0) {
      throw std::invalid_argument("orientation parameters must be valid");
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
  static Quaternion normalizeQuaternion(const Quaternion &quaternion) {
    const double magnitude = std::sqrt(
        quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

    if (!std::isfinite(magnitude) || magnitude <= 1.0e-9) {
      return {0.0, 0.0, 0.0, 1.0};
    }

    return {
        quaternion[0] / magnitude,
        quaternion[1] / magnitude,
        quaternion[2] / magnitude,
        quaternion[3] / magnitude,
    };
  }

  static Quaternion conjugateQuaternion(const Quaternion &quaternion) {
    return {
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    };
  }

  static Quaternion multiplyQuaternions(const Quaternion &first,
                                        const Quaternion &second) {
    return normalizeQuaternion({
        first[3] * second[0] + first[0] * second[3] + first[1] * second[2] -
            first[2] * second[1],
        first[3] * second[1] - first[0] * second[2] + first[1] * second[3] +
            first[2] * second[0],
        first[3] * second[2] + first[0] * second[1] - first[1] * second[0] +
            first[2] * second[3],
        first[3] * second[3] - first[0] * second[0] - first[1] * second[1] -
            first[2] * second[2],
    });
  }

  static double quaternionDot(const Quaternion &first,
                              const Quaternion &second) {
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2] +
           first[3] * second[3];
  }

  static Point quaternionRotationVector(Quaternion quaternion) {
    quaternion = normalizeQuaternion(quaternion);

    if (quaternion[3] < 0.0) {
      for (double &value : quaternion) {
        value = -value;
      }
    }

    const double vector_magnitude = std::sqrt(quaternion[0] * quaternion[0] +
                                              quaternion[1] * quaternion[1] +
                                              quaternion[2] * quaternion[2]);

    if (vector_magnitude <= 1.0e-9) {
      return {0.0, 0.0, 0.0};
    }

    const double angle = 2.0 * std::atan2(vector_magnitude, quaternion[3]);

    const double scale = angle / vector_magnitude;

    return {
        quaternion[0] * scale,
        quaternion[1] * scale,
        quaternion[2] * scale,
    };
  }

  static Quaternion quaternionFromRollPitchYaw(double roll, double pitch,
                                               double yaw) {
    const double cr = std::cos(roll / 2.0);
    const double sr = std::sin(roll / 2.0);
    const double cp = std::cos(pitch / 2.0);
    const double sp = std::sin(pitch / 2.0);
    const double cy = std::cos(yaw / 2.0);
    const double sy = std::sin(yaw / 2.0);

    return normalizeQuaternion({
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    });
  }

  double mappedAngle(double angle) const {
    if (std::abs(angle) < orientation_deadband_) {
      return 0.0;
    }

    return std::clamp(angle, -maximum_orientation_offset_,
                      maximum_orientation_offset_);
  }

  void resetTracking() {
    latest_relative_depth_ = 0.0;
    has_latest_palm_center_ = false;
    has_latest_palm_orientation_ = false;
    orientation_reference_initialized_ = false;
    missed_frame_count_ = 0;
    depth_valid_ = false;
  }

  void handleTrackingLoss() {
    has_latest_palm_center_ = false;
    has_latest_palm_orientation_ = false;
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
        !std::isfinite(message->palm_orientation.x) ||
        !std::isfinite(message->palm_orientation.y) ||
        !std::isfinite(message->palm_orientation.z) ||
        !std::isfinite(message->palm_orientation.w) ||
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

    if (!message->orientation_valid) {
      has_latest_palm_orientation_ = false;
      return;
    }

    latest_palm_orientation_ = normalizeQuaternion({
        message->palm_orientation.x,
        message->palm_orientation.y,
        message->palm_orientation.z,
        message->palm_orientation.w,
    });

    has_latest_palm_orientation_ = true;

    if (!orientation_reference_initialized_) {
      reference_palm_orientation_ = latest_palm_orientation_;

      orientation_reference_initialized_ = true;

      RCLCPP_INFO(get_logger(), "Cartesian orientation reference initialized");
    }
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
            ? std::clamp(latest_relative_depth_ * workspace_x_scale_,
                         -maximum_x_offset_, maximum_x_offset_)
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

    if (has_latest_palm_orientation_) {
      const Quaternion relative_hand_orientation =
          multiplyQuaternions(conjugateQuaternion(reference_palm_orientation_),
                              latest_palm_orientation_);

      const Point hand_rotation =
          quaternionRotationVector(relative_hand_orientation);

      const double hand_roll = mappedAngle(-hand_rotation[2]);

      const double hand_pitch = mappedAngle(hand_rotation[0]);

      const double hand_yaw = mappedAngle(-hand_rotation[1]);

      const Quaternion hand_frame_offset =
          quaternionFromRollPitchYaw(hand_roll, hand_pitch, hand_yaw);

      const Quaternion ee_to_hand = normalizeQuaternion({
          -0.5,
          0.5,
          -0.5,
          0.5,
      });

      const Quaternion ee_frame_offset = multiplyQuaternions(
          multiplyQuaternions(ee_to_hand, hand_frame_offset),
          conjugateQuaternion(ee_to_hand));

      const Quaternion reference_orientation = {
          reference_orientation_x_,
          reference_orientation_y_,
          reference_orientation_z_,
          reference_orientation_w_,
      };

      const Quaternion target_orientation =
          multiplyQuaternions(reference_orientation, ee_frame_offset);

      target.pose.orientation.x = target_orientation[0];

      target.pose.orientation.y = target_orientation[1];

      target.pose.orientation.z = target_orientation[2];

      target.pose.orientation.w = target_orientation[3];
    }

    const double x_change =
        std::abs(target.pose.position.x - last_target_.pose.position.x);

    const double y_change =
        std::abs(target.pose.position.y - last_target_.pose.position.y);

    const double z_change =
        std::abs(target.pose.position.z - last_target_.pose.position.z);

    const Quaternion target_orientation = {
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w,
    };

    const Quaternion previous_orientation = {
        last_target_.pose.orientation.x,
        last_target_.pose.orientation.y,
        last_target_.pose.orientation.z,
        last_target_.pose.orientation.w,
    };

    const double orientation_change =
        2.0 *
        std::acos(std::clamp(
            std::abs(quaternionDot(target_orientation, previous_orientation)),
            0.0, 1.0));

    if (x_change < minimum_target_change_ &&
        y_change < minimum_target_change_ &&
        z_change < minimum_target_change_ &&
        orientation_change < minimum_orientation_change_) {
      return;
    }

    target_publisher_->publish(target);
    last_target_ = target;
  }

  std::string base_frame_;
  std::string tip_link_;

  double workspace_x_scale_;
  bool enable_depth_control_;
  double workspace_y_scale_;
  double workspace_z_scale_;
  double workspace_z_upward_scale_;
  double maximum_x_offset_;
  double screen_center_x_;
  double screen_center_y_;
  int maximum_missed_frames_;
  double maximum_y_offset_;
  double maximum_z_upward_offset_;
  double maximum_z_downward_offset_;
  double minimum_target_change_;
  double maximum_orientation_offset_;
  double orientation_deadband_;
  double minimum_orientation_change_;

  double reference_x_;
  double reference_y_;
  double reference_z_;
  double reference_orientation_x_;
  double reference_orientation_y_;
  double reference_orientation_z_;
  double reference_orientation_w_;

  Point latest_palm_center_;
  Quaternion latest_palm_orientation_;
  Quaternion reference_palm_orientation_;
  double latest_relative_depth_;

  geometry_msgs::msg::PoseStamped last_target_;

  bool has_latest_palm_center_;
  bool has_latest_palm_orientation_;
  bool orientation_reference_initialized_;
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
