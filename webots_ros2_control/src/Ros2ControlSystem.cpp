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
  void Ros2ControlSystem::init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info)
  {
    mNode = node;
    for (hardware_interface::ComponentInfo component : info.joints)
    {
      Joint joint;
      joint.name = component.name;

      webots::Motor *motor = mNode->robot()->getMotor(joint.name);
      webots::PositionSensor *sensor = mNode->robot()->getPositionSensor(joint.name);
      joint.motor = (motor) ? motor : sensor->getMotor();
      joint.sensor = (sensor) ? sensor : motor->getPositionSensor();
      if (joint.sensor)
        joint.sensor->enable(node->robot()->getBasicTimeStep());
      if (!joint.sensor && !joint.motor)
        throw std::runtime_error("Cannot find a Motor or PositionSensor with name " + joint.name);

      joint.controlPosition = false;
      joint.controlVelocity = false;
      joint.controlEffort = false;
      joint.positionCommand = NAN;
      joint.velocityCommand = NAN;
      joint.effortCommand = NAN;
      for (hardware_interface::InterfaceInfo commandInterface : component.command_interfaces)
      {
        if (commandInterface.name == "position")
          joint.controlPosition = true;
        else if (commandInterface.name == "velocity")
        {
          joint.controlVelocity = true;
          if (joint.motor)
          {
            joint.motor->setPosition(INFINITY);
            joint.motor->setVelocity(0.0);
          }
        }
        else if (commandInterface.name == "effort")
          joint.controlEffort = true;
        else
          throw std::runtime_error("Invalid hardware info name `" + commandInterface.name + "`");
      }

      mJoints.push_back(joint);
    }
  }

  #if FOXY
    hardware_interface::return_type Ros2ControlSystem::configure(const hardware_interface::HardwareInfo &info)
    {
      if (configure_default(info) != hardware_interface::return_type::OK)
      {
        return hardware_interface::return_type::ERROR;
      }
      status_ = hardware_interface::status::CONFIGURED;
      return hardware_interface::return_type::OK;
    }
  #else
    CallbackReturn Ros2ControlSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
      if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      {
        return CallbackReturn::ERROR;
      }
      return CallbackReturn::SUCCESS;
    }
  #endif

  std::vector<hardware_interface::StateInterface> Ros2ControlSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (Joint &joint : mJoints)
      if (joint.sensor)
        interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Ros2ControlSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (Joint &joint : mJoints)
      if (joint.motor)
      {
        if (joint.controlPosition)
          interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));
        if (joint.controlEffort)
          interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));
        if (joint.controlVelocity)
          interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));
      }
    return interfaces;
  }

  #if FOXY
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
  #else
    CallbackReturn Ros2ControlSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn Ros2ControlSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      return CallbackReturn::SUCCESS;
    }
  #endif

  hardware_interface::return_type Ros2ControlSystem::read()
  {
    for (Joint &joint : mJoints)
    {
      if (joint.sensor)
        joint.position = joint.sensor->getValue();
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::write()
  {
    for (Joint &joint : mJoints)
    {
      if (joint.motor)
      {
        if (joint.controlPosition && !std::isnan(joint.positionCommand))
          joint.motor->setPosition(joint.positionCommand);
        if (joint.controlVelocity && !std::isnan(joint.velocityCommand))
          joint.motor->setVelocity(joint.velocityCommand);
        if (joint.controlEffort && !std::isnan(joint.effortCommand))
          joint.motor->setTorque(joint.effortCommand);
      }
    }
    return hardware_interface::return_type::OK;
  }
}

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2ControlSystem, webots_ros2_control::Ros2ControlSystemInterface)
