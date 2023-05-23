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

#ifndef ROS2_CONTROL_INTERFACE_HPP
#define ROS2_CONTROL_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include <webots/supervisor.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace webots_ros2_control {
  class Ros2ControlSystemInterface : public hardware_interface::SystemInterface {
  public:
    virtual void init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) = 0;
  };
}  // namespace webots_ros2_control

#endif
