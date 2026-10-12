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

#include "webots_ros2_control/Ros2ControlSystem.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <webots/device.h>
#include <webots/robot.h>

namespace webots_ros2_control {
  Ros2ControlSystem::Ros2ControlSystem() {
    mNode = NULL;
  }
  void Ros2ControlSystem::init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) {
    mNode = node;
    for (hardware_interface::ComponentInfo component : info.joints) {
      Joint joint;
      joint.name = component.name;

      WbDeviceTag device = wb_robot_get_device(joint.name.c_str());
      WbNodeType type = wb_device_get_node_type(device);
      joint.motor =
        (type == WB_NODE_LINEAR_MOTOR || type == WB_NODE_ROTATIONAL_MOTOR) ? device : wb_position_sensor_get_motor(device);
      device = (component.parameters.count("sensor") == 0) ? wb_robot_get_device(joint.name.c_str()) :
                                                             wb_robot_get_device(component.parameters.at("sensor").c_str());
      type = wb_device_get_node_type(device);
      joint.sensor = (type == WB_NODE_POSITION_SENSOR) ? device : wb_motor_get_position_sensor(device);

      if (joint.sensor)
        wb_position_sensor_enable(joint.sensor, wb_robot_get_basic_time_step());
      if (!joint.sensor && !joint.motor)
        throw std::runtime_error("Cannot find a Motor or PositionSensor with name " + joint.name);

      // Initialize the state
      joint.controlPosition = false;
      joint.controlVelocity = false;
      joint.controlEffort = false;
      joint.position = NAN;
      joint.velocity = NAN;
      joint.acceleration = NAN;

      // Check if state interfaces have initial positions
      for (hardware_interface::InterfaceInfo stateInterface : component.state_interfaces) {
        if (stateInterface.name == "position" && !stateInterface.initial_value.empty()) {
          joint.position = std::stod(stateInterface.initial_value);
          wb_motor_set_position(joint.motor, std::stod(stateInterface.initial_value));
        }
      }

      // Configure the command interface
      for (hardware_interface::InterfaceInfo commandInterface : component.command_interfaces) {
        if (commandInterface.name == "position")
          joint.controlPosition = true;
        else if (commandInterface.name == "velocity")
          joint.controlVelocity = true;
        else if (commandInterface.name == "effort")
          joint.controlEffort = true;
        else
          throw std::runtime_error("Invalid hardware info name `" + commandInterface.name + "`");
      }
      if (joint.motor && joint.controlVelocity && !joint.controlPosition) {
        wb_motor_set_position(joint.motor, INFINITY);
        wb_motor_set_velocity(joint.motor, 0.0);
      }

      // Create the state and command interface handles up front. The resource manager keeps
      // shared ownership of these; read()/write() update their values through set_value()/get_value().
      if (joint.sensor) {
        joint.position_state = std::make_shared<hardware_interface::StateInterface>(
          joint.name, hardware_interface::HW_IF_POSITION);
        joint.velocity_state = std::make_shared<hardware_interface::StateInterface>(
          joint.name, hardware_interface::HW_IF_VELOCITY);
        joint.acceleration_state = std::make_shared<hardware_interface::StateInterface>(
          joint.name, hardware_interface::HW_IF_ACCELERATION);
        (void)joint.position_state->set_value(joint.position, true);
        (void)joint.velocity_state->set_value(joint.velocity, true);
        (void)joint.acceleration_state->set_value(joint.acceleration, true);
      }
      if (joint.motor) {
        if (joint.controlPosition) {
          joint.position_command = std::make_shared<hardware_interface::CommandInterface>(
            joint.name, hardware_interface::HW_IF_POSITION);
          (void)joint.position_command->set_value(std::numeric_limits<double>::quiet_NaN(), true);
        }
        if (joint.controlEffort) {
          joint.effort_command = std::make_shared<hardware_interface::CommandInterface>(
            joint.name, hardware_interface::HW_IF_EFFORT);
          (void)joint.effort_command->set_value(std::numeric_limits<double>::quiet_NaN(), true);
        }
        if (joint.controlVelocity) {
          joint.velocity_command = std::make_shared<hardware_interface::CommandInterface>(
            joint.name, hardware_interface::HW_IF_VELOCITY);
          (void)joint.velocity_command->set_value(std::numeric_limits<double>::quiet_NaN(), true);
        }
      }

      mJoints.push_back(joint);
    }
  }

#if HARDWARE_INTERFACE_VERSION_MAJOR > 5 || (HARDWARE_INTERFACE_VERSION_MAJOR == 5 && HARDWARE_INTERFACE_VERSION_MINOR >= 3)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#else
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> Ros2ControlSystem::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
    for (Joint &joint : mJoints)
      if (joint.sensor) {
        interfaces.push_back(joint.position_state);
        interfaces.push_back(joint.velocity_state);
        interfaces.push_back(joint.acceleration_state);
      }

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> Ros2ControlSystem::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
    for (Joint &joint : mJoints)
      if (joint.motor) {
        if (joint.controlPosition)
          interfaces.push_back(joint.position_command);
        if (joint.controlEffort)
          interfaces.push_back(joint.effort_command);
        if (joint.controlVelocity)
          interfaces.push_back(joint.velocity_command);
      }
    return interfaces;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Ros2ControlSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    static double lastReadTime = 0;

    const double deltaTime = wb_robot_get_time() - lastReadTime;
    lastReadTime = wb_robot_get_time();

    for (Joint &joint : mJoints) {
      if (joint.sensor) {
        const double position = wb_position_sensor_get_value(joint.sensor);
        const double velocity = std::isnan(joint.position) ? NAN : (position - joint.position) / deltaTime;

        if (!std::isnan(joint.velocity))
          joint.acceleration = (velocity - joint.velocity) / deltaTime;
        joint.velocity = velocity;
        joint.position = position;

        (void)joint.position_state->set_value(joint.position, true);
        (void)joint.velocity_state->set_value(joint.velocity, true);
        (void)joint.acceleration_state->set_value(joint.acceleration, true);
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    for (Joint &joint : mJoints) {
      if (joint.motor) {
        double command;
        if (joint.controlPosition && joint.position_command->get_value(command, true) && !std::isnan(command))
          wb_motor_set_position(joint.motor, command);
        if (joint.controlVelocity && joint.velocity_command->get_value(command, true) && !std::isnan(command)) {
          // In the position control mode the velocity cannot be negative.
          const double velocityCommand = joint.controlPosition ? abs(command) : command;
          wb_motor_set_velocity(joint.motor, velocityCommand);
        }
        if (joint.controlEffort && joint.effort_command->get_value(command, true) && !std::isnan(command))
          wb_motor_set_torque(joint.motor, command);
      }
    }
    return hardware_interface::return_type::OK;
  }
}  // namespace webots_ros2_control

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2ControlSystem, webots_ros2_control::Ros2ControlSystemInterface)
