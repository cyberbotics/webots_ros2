// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "webots_ros2_control/Ros2Control.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <iostream>
namespace webots_ros2_control
{

  void Ros2Control::step()
  {
    std::cout << "Ros2Control::step()\n";
  }

  void Ros2Control::init(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters)
  {
    std::cout << "Ros2Control::init()\n";
    mNode = node;
  }
}

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2Control, webots_ros2::PluginInterface)
