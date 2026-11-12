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

#ifndef ROBOT_STATE_ESTIMATION__PALM_STATE_EXTRACTOR_HPP_
#define ROBOT_STATE_ESTIMATION__PALM_STATE_EXTRACTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace robot_state_estimation {

struct PalmFeatures {
  using Point = std::array<double, 3>;

  Point center;
  Point normal;
  double scale;
  bool depth_valid;
};

class PalmStateExtractor {
public:
  static constexpr std::size_t kLandmarkValueCount = 63U;
  static constexpr double kDefaultMinimumPalmNormalZ = 0.70;

  using Landmarks = std::array<double, kLandmarkValueCount>;
  using Point = PalmFeatures::Point;

  static PalmFeatures
  extract(const Landmarks &landmarks,
          double minimum_palm_normal_z = kDefaultMinimumPalmNormalZ) {
    validateMinimumPalmNormalZ(minimum_palm_normal_z);
    validateLandmarks(landmarks);

    const Point wrist = point(landmarks, 0U);
    const Point index_mcp = point(landmarks, 5U);
    const Point middle_mcp = point(landmarks, 9U);
    const Point ring_mcp = point(landmarks, 13U);
    const Point pinky_mcp = point(landmarks, 17U);

    const Point center = {
        (wrist[0] + index_mcp[0] + middle_mcp[0] + ring_mcp[0] + pinky_mcp[0]) /
            5.0,
        (wrist[1] + index_mcp[1] + middle_mcp[1] + ring_mcp[1] + pinky_mcp[1]) /
            5.0,
        (wrist[2] + index_mcp[2] + middle_mcp[2] + ring_mcp[2] + pinky_mcp[2]) /
            5.0,
    };

    std::array<double, 4> palm_distances = {
        distance(index_mcp, pinky_mcp),
        distance(wrist, middle_mcp),
        distance(index_mcp, ring_mcp),
        distance(middle_mcp, pinky_mcp),
    };

    std::sort(palm_distances.begin(), palm_distances.end());

    const double scale = (palm_distances[1] + palm_distances[2]) / 2.0;

    const Point across_palm = subtract(index_mcp, pinky_mcp);

    const Point along_palm = subtract(middle_mcp, wrist);

    const Point normal = normalize(cross(across_palm, along_palm));

    const bool valid_scale = std::isfinite(scale) && scale > kMinimumMagnitude;

    const bool valid_normal = norm(normal) > kMinimumMagnitude;

    const bool depth_valid = valid_scale && valid_normal &&
                             std::abs(normal[2]) >= minimum_palm_normal_z;

    return {
        center,
        normal,
        scale,
        depth_valid,
    };
  }

private:
  static constexpr double kMinimumMagnitude = 1.0e-9;

  static void validateMinimumPalmNormalZ(double minimum_palm_normal_z) {
    if (!std::isfinite(minimum_palm_normal_z) || minimum_palm_normal_z < 0.0 ||
        minimum_palm_normal_z > 1.0) {
      throw std::invalid_argument("minimum_palm_normal_z must be finite "
                                  "and within [0, 1]");
    }
  }

  static void validateLandmarks(const Landmarks &landmarks) {
    for (const double value : landmarks) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "landmarks must contain only finite values");
      }
    }
  }

  static Point point(const Landmarks &landmarks, std::size_t index) {
    const std::size_t offset = index * 3U;

    return {
        landmarks[offset],
        landmarks[offset + 1U],
        landmarks[offset + 2U],
    };
  }

  static Point subtract(const Point &first, const Point &second) {
    return {
        first[0] - second[0],
        first[1] - second[1],
        first[2] - second[2],
    };
  }

  static double dot(const Point &first, const Point &second) {
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
  }

  static double norm(const Point &value) {
    return std::sqrt(dot(value, value));
  }

  static double distance(const Point &first, const Point &second) {
    return norm(subtract(first, second));
  }

  static Point cross(const Point &first, const Point &second) {
    return {
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    };
  }

  static Point normalize(const Point &value) {
    const double magnitude = norm(value);

    if (magnitude <= kMinimumMagnitude) {
      return {0.0, 0.0, 0.0};
    }

    return {
        value[0] / magnitude,
        value[1] / magnitude,
        value[2] / magnitude,
    };
  }
};

} // namespace robot_state_estimation

#endif // ROBOT_STATE_ESTIMATION__PALM_STATE_EXTRACTOR_HPP_
