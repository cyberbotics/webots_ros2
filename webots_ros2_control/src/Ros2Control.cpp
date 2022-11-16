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
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <webots/robot.h>

#include <iostream>

const double CONTROLLER_MANAGER_ALLOWED_SAMPLE_ERROR_MS = 1.0;

namespace webots_ros2_control
{

  void Ros2Control::step()
  {
    const int nowMs = wb_robot_get_time() * 1000.0;
    const int periodMs = nowMs - mLastControlUpdateMs;
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(mControlPeriodMs / 1000.0);
    if (periodMs >= mControlPeriodMs)
    {
#if FOXY || GALACTIC
      mControllerManager->read();
#else
      mControllerManager->read(mNode->get_clock()->now(), dt);
#endif

#if FOXY
      mControllerManager->update();
#else
      mControllerManager->update(mNode->get_clock()->now(), dt);
      mLastControlUpdateMs = nowMs;
#endif

#if FOXY || GALACTIC
    mControllerManager->write();
#else // HUMBLE, ROLLING
    mControllerManager->write(mNode->get_clock()->now(), dt);
#endif
    }
  }

  void Ros2Control::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &)
  {
    mNode = node;
    mLastControlUpdateMs = 0;

    // Load hardware
    try
    {
      mHardwareLoader.reset(new pluginlib::ClassLoader<webots_ros2_control::Ros2ControlSystemInterface>(
          "webots_ros2_control",
          "webots_ros2_control::Ros2ControlSystemInterface"));
    }
    catch (pluginlib::LibraryLoadException &ex)
    {
      throw std::runtime_error("Hardware loader cannot be created: " + atoi(ex.what()));
    }

    // Control Hardware
    std::string urdfString;
    std::vector<hardware_interface::HardwareInfo> controlHardware;
    std::unique_ptr<hardware_interface::ResourceManager> resourceManager = std::make_unique<hardware_interface::ResourceManager>();
    try
    {
      urdfString = mNode->urdf();
      controlHardware = hardware_interface::parse_control_resources_from_urdf(urdfString);
    }
    catch (const std::runtime_error &ex)
    {
      throw std::runtime_error("URDF cannot be parsed by a `ros2_control` component parser: " + atoi(ex.what()));
    }
    for (unsigned int i = 0; i < controlHardware.size(); i++)
    {
      const std::string hardwareType = controlHardware[i].hardware_class_type;
      auto webotsSystem = std::unique_ptr<webots_ros2_control::Ros2ControlSystemInterface>(
          mHardwareLoader->createUnmanagedInstance(hardwareType));
      webotsSystem->init(mNode, controlHardware[i]);
#if FOXY
      resourceManager->import_component(std::move(webotsSystem));
#else
      resourceManager->import_component(std::move(webotsSystem), controlHardware[i]);
#endif

#if HUMBLE || ROLLING
      resourceManager->activate_all_components();
#endif
    }

    // Controller Manager
    mExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    mControllerManager.reset(new controller_manager::ControllerManager(std::move(resourceManager), mExecutor));

    // Update rate
    const int updateRate = mControllerManager->get_parameter("update_rate").as_int();
    mControlPeriodMs = (1.0 / updateRate) * 1000.0;

    int controlPeriodProductMs = wb_robot_get_basic_time_step();
    while (controlPeriodProductMs < mControlPeriodMs)
      controlPeriodProductMs += wb_robot_get_basic_time_step();

    if (abs(controlPeriodProductMs - mControlPeriodMs) > CONTROLLER_MANAGER_ALLOWED_SAMPLE_ERROR_MS)
      RCLCPP_WARN_STREAM(node->get_logger(), "Desired controller update period (" << mControlPeriodMs << "ms / " << updateRate << "Hz) is different from the Webots timestep (" << wb_robot_get_basic_time_step() << "ms). Please adjust the `update_rate` parameter in the `controller_manager` or the `basicTimeStep` parameter in the Webots `WorldInfo` node.");

    // Spin
    mExecutor->add_node(mControllerManager);
    auto spin = [this]()
    {
      while (rclcpp::ok())
        mExecutor->spin_once();
    };
    mThreadExecutor = std::thread(spin);
  }
}

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2Control, webots_ros2_driver::PluginInterface)
