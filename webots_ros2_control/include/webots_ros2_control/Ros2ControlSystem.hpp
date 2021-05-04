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

#ifndef ROS2_CONTROL_HPP
#define ROS2_CONTROL_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "webots_ros2_cpp/PluginInterface.hpp"

namespace webots_ros2_control
{
  struct Ros2ControlInfo
  {
    std::vector<double> jointPositionsCommand;
    std::vector<double> jointPositions;
  };

  class Ros2ControlSystem : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
  {
  public:
    // hardware_interface::BaseInterface
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;

  private:
    std::vector<double> mCommands;
    std::vector<double> mStates;
    Ros2ControlInfo mInfo;
  };
}

#endif
