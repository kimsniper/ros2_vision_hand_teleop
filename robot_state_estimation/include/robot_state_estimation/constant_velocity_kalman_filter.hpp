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

#ifndef ROBOT_STATE_ESTIMATION__CONSTANT_VELOCITY_KALMAN_FILTER_HPP_
#define ROBOT_STATE_ESTIMATION__CONSTANT_VELOCITY_KALMAN_FILTER_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_state_estimation {

class ConstantVelocityKalmanFilter {
public:
  ConstantVelocityKalmanFilter(double process_noise, double measurement_noise,
                               double initial_position_variance,
                               double initial_velocity_variance,
                               double velocity_damping = 1.0)
      : process_noise_(requirePositive(process_noise, "process_noise")),
        measurement_noise_(
            requirePositive(measurement_noise, "measurement_noise")),
        initial_position_variance_(requirePositive(
            initial_position_variance, "initial_position_variance")),
        initial_velocity_variance_(requirePositive(
            initial_velocity_variance, "initial_velocity_variance")),
        velocity_damping_(requireDamping(velocity_damping)) {
    reset();
  }

  void initialize(double position) {
    requireFinite(position, "position");

    position_ = position;
    velocity_ = 0.0;

    position_variance_ = initial_position_variance_;
    position_velocity_covariance_ = 0.0;
    velocity_variance_ = initial_velocity_variance_;

    initialized_ = true;
  }

  double update(double measurement, double dt) {
    requireFinite(measurement, "measurement");
    requirePositive(dt, "dt");

    if (!initialized_) {
      initialize(measurement);
      return position_;
    }

    const double predicted_velocity = velocity_ * velocity_damping_;

    const double predicted_position = position_ + predicted_velocity * dt;

    const double dt_squared = dt * dt;
    const double dt_cubed = dt_squared * dt;
    const double dt_fourth = dt_squared * dt_squared;

    const double predicted_position_variance =
        position_variance_ + 2.0 * dt * position_velocity_covariance_ +
        dt_squared * velocity_variance_ + 0.25 * dt_fourth * process_noise_;

    const double predicted_position_velocity_covariance =
        position_velocity_covariance_ + dt * velocity_variance_ +
        0.5 * dt_cubed * process_noise_;

    const double predicted_velocity_variance =
        velocity_variance_ + dt_squared * process_noise_;

    const double innovation = measurement - predicted_position;
    const double innovation_variance =
        predicted_position_variance + measurement_noise_;

    const double position_gain =
        predicted_position_variance / innovation_variance;

    const double velocity_gain =
        predicted_position_velocity_covariance / innovation_variance;

    position_ = predicted_position + position_gain * innovation;
    velocity_ = predicted_velocity + velocity_gain * innovation;

    position_variance_ = (1.0 - position_gain) * predicted_position_variance;

    position_velocity_covariance_ =
        (1.0 - position_gain) * predicted_position_velocity_covariance;

    velocity_variance_ = predicted_velocity_variance -
                         velocity_gain * predicted_position_velocity_covariance;

    return position_;
  }

  void reset() {
    position_ = 0.0;
    velocity_ = 0.0;

    position_variance_ = 0.0;
    position_velocity_covariance_ = 0.0;
    velocity_variance_ = 0.0;

    initialized_ = false;
  }

  bool initialized() const { return initialized_; }

  double position() const { return position_; }

  double velocity() const { return velocity_; }

  double positionVariance() const { return position_variance_; }

private:
  static void requireFinite(double value, const char *name) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string(name) + " must be finite");
    }
  }

  static double requirePositive(double value, const char *name) {
    requireFinite(value, name);

    if (value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be positive");
    }

    return value;
  }

  static double requireDamping(double value) {
    requireFinite(value, "velocity_damping");

    if (value < 0.0 || value > 1.0) {
      throw std::invalid_argument("velocity_damping must be within [0, 1]");
    }

    return value;
  }

  double process_noise_;
  double measurement_noise_;
  double initial_position_variance_;
  double initial_velocity_variance_;
  double velocity_damping_;

  double position_;
  double velocity_;

  double position_variance_;
  double position_velocity_covariance_;
  double velocity_variance_;

  bool initialized_;
};

} // namespace robot_state_estimation

#endif // ROBOT_STATE_ESTIMATION__CONSTANT_VELOCITY_KALMAN_FILTER_HPP_
