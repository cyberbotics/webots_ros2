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

#ifndef ROS2_CONTROL_HPP
#define ROS2_CONTROL_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "webots/motor.h"
#include "webots/position_sensor.h"
#include "webots_ros2_control/Ros2ControlSystemInterface.hpp"

namespace webots_ros2_control {
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
    WbDeviceTag motor;
    WbDeviceTag sensor;
  };

  class Ros2ControlSystem : public Ros2ControlSystemInterface {
  public:
    Ros2ControlSystem();
    void init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  private:
    webots_ros2_driver::WebotsNode *mNode;
    std::vector<Joint> mJoints;
  };
}  // namespace webots_ros2_control

#endif
