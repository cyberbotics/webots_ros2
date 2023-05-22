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

#ifndef MATH_HPP
#define MATH_HPP

#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace webots_ros2_driver {
  void matrixToQuaternion(const double *matrix, geometry_msgs::msg::Quaternion &q);
  void axisAngleToQuaternion(const double *axisAngle, geometry_msgs::msg::Quaternion &q);
  void quaternionToAxisAngle(const geometry_msgs::msg::Quaternion &q, double *axisAngle);

  double interpolateLookupTable(double value, const std::vector<double> &table);
}  // namespace webots_ros2_driver
#endif
