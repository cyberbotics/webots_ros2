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

#if FOXY
  #include "hardware_interface/base_interface.hpp"
  #include "hardware_interface/types/hardware_interface_status_values.hpp"
#endif

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "webots_ros2_control/Ros2ControlSystemInterface.hpp"
#include "webots/Motor.hpp"
#include "webots/PositionSensor.hpp"

namespace webots_ros2_control
{
  struct Joint {
    double positionCommand;
    double position;
    double velocityCommand;
    double velocity;
    double effortCommand;
    double acceleration;
    bool controlPosition;
    bool controlVelocity;
    bool controlEffort;
    std::string name;
    webots::Motor* motor;
    webots::PositionSensor* sensor;
  };

  class Ros2ControlSystem : public Ros2ControlSystemInterface
  {
  public:
    void init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) override;

    #if FOXY
      hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
      hardware_interface::return_type start() override;
      hardware_interface::return_type stop() override;
    #else
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    #endif

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#if FOXY || GALACTIC || HUMBLE
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;
#else  // ROLLING
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
#endif

  private:
    webots_ros2_driver::WebotsNode *mNode;
    std::vector<Joint> mJoints;
  };
}

#endif
