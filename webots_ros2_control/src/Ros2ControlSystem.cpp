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

#include "webots_ros2_control/Ros2ControlSystem.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace webots_ros2_control
{
  void Ros2ControlSystem::init(webots_ros2::WebotsNode *node)
  {
  }

  hardware_interface::return_type Ros2ControlSystem::configure(
      const hardware_interface::HardwareInfo &info)
  {
    status_ = hardware_interface::status::CONFIGURED;

    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  Ros2ControlSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back(
        hardware_interface::StateInterface(
            "interfaceName",
            hardware_interface::HW_IF_POSITION, &mInfo.jointPositions[0]));

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Ros2ControlSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(
        hardware_interface::CommandInterface(
            "interfaceName",
            hardware_interface::HW_IF_POSITION, &mInfo.jointPositionsCommand[0]));

    return interfaces;
  }

  hardware_interface::return_type Ros2ControlSystem::start()
  {
    status_ = hardware_interface::status::STARTED;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::stop()
  {
    status_ = hardware_interface::status::STOPPED;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::read()
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::write()
  {
    return hardware_interface::return_type::OK;
  }
}

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2ControlSystem, webots_ros2_control::Ros2ControlSystemInterface)
