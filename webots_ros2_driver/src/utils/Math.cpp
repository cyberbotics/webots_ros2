// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <webots_ros2_driver/utils/Math.hpp>

namespace webots_ros2_driver {
  static double interpolateFunction(double value, double startX, double startY, double endX, double endY,
                                    bool ascending = false);

  void matrixToQuaternion(const double *matrix, geometry_msgs::msg::Quaternion &q) {
    if (matrix[0] == matrix[4] && matrix[0] == matrix[8] && matrix[0] == 1.0) {
      // exception
      q.w = 1.0;
      q.x = 0.0;
      q.y = 0.0;
      q.z = 0.0;
      return;
    }

    double s = 2.0;
    double invS = 1.0;
    const double trace = matrix[0] + matrix[4] + matrix[8];
    if (trace >= 0.0) {
      s *= sqrt(trace + 1);
      invS = 1.0 / s;
      q.w = 0.25 * s;
      q.x = (matrix[7] - matrix[5]) * invS;
      q.y = (matrix[2] - matrix[6]) * invS;
      q.z = (matrix[3] - matrix[1]) * invS;
      return;
    }

    if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
      // matrix[0] is larger than max(M(4), M(8))
      s *= sqrt(1.0 + matrix[0] - matrix[4] - matrix[8]);
      invS = 1.0 / s;
      q.w = (matrix[7] - matrix[5]) * invS;
      q.x = 0.25 * s;
      q.y = (matrix[1] + matrix[3]) * invS;
      q.z = (matrix[6] + matrix[2]) * invS;
      return;
    }

    if (matrix[4] > matrix[8]) {
      // matrix[4] is the largest
      s *= sqrt(1.0 + matrix[4] - matrix[8] - matrix[0]);  // s = 4y
      invS = 1.0 / s;
      q.w = (matrix[2] - matrix[6]) * invS;
      q.x = (matrix[1] + matrix[3]) * invS;
      q.y = 0.25 * s;
      q.z = (matrix[5] + matrix[7]) * invS;
      return;
    }

    // else matrix[8] is the largest
    s *= sqrt(1.0 + matrix[8] - matrix[0] - matrix[4]);  // s = 4z
    invS = 1.0 / s;
    q.w = (matrix[3] - matrix[1]) * invS;
    q.x = (matrix[6] + matrix[2]) * invS;
    q.y = (matrix[5] + matrix[7]) * invS;
    q.z = 0.25 * s;
  }

  void axisAngleToQuaternion(const double *axisAngle, geometry_msgs::msg::Quaternion &q) {
    const double halfAngle = 0.5 * axisAngle[3];
    const double sinusHalfAngle = sin(halfAngle);
    q.w = cos(halfAngle);
    q.x = axisAngle[0] * sinusHalfAngle;
    q.y = axisAngle[1] * sinusHalfAngle;
    q.z = axisAngle[2] * sinusHalfAngle;
  }

  void quaternionToAxisAngle(const geometry_msgs::msg::Quaternion &q, double *axisAngle) {
    // if q.w > 1, acos will return nan
    // if this actually happens we should normalize the quaternion here
    axisAngle[3] = 2.0 * acos(q.w);
    if (axisAngle[3] < 0.0001) {
      axisAngle[0] = 0.0;
      axisAngle[1] = 1.0;
      axisAngle[2] = 0.0;
      axisAngle[3] = 0.0;
    }
    // normalise axes
    const double inv = 1.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    axisAngle[0] = q.x * inv;
    axisAngle[1] = q.y * inv;
    axisAngle[2] = q.z * inv;
  }

  double interpolateFunction(double value, double startX, double startY, double endX, double endY, bool ascending) {
    if (endX - startX == 0)
      if ((ascending && value < startX) || (!ascending && value > startX))
        return startY;
      else if ((ascending && value > startX) || (!ascending && value < startX))
        return endY;
      else
        return startY + (endY - startY) / 2;
    const double slope = (endY - startY) / (endX - startX);
    return slope * (value - startX) + startY;
  }

  double interpolateLookupTable(double value, const std::vector<double> &table) {
    if (!table.size())
      return value;
    const int size = table.size();

    // Interpolate
    for (int i = 0; i < size / 3 - 1; i++) {
      if ((value < table[i * 3 + 1] && value >= table[(i + 1) * 3 + 1]) ||
          (value >= table[i * 3 + 1] && value < table[(i + 1) * 3 + 1]))
        return interpolateFunction(value, table[i * 3 + 1], table[i * 3], table[(i + 1) * 3 + 1], table[(i + 1) * 3]);
    }

    // Extrapolate (we assume that the table is sorted, order is irrelevant)
    const bool ascending = (table[1] < table[size - 1 * 3 + 1]);
    if ((ascending && value >= table[1]) || (!ascending && value < table[1]))
      return interpolateFunction(value, table[size - 2 * 3 + 1], table[size - 2 * 3 + 0], table[size - 1 * 3 + 1],
                                 table[size - 1 * 3 + 0], ascending);
    else
      return interpolateFunction(value, table[1], table[0], table[3 + 1], table[3], ascending);
  }
}  // namespace webots_ros2_driver
