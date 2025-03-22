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

#include "webots_ros2_control/Ros2Control.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <webots/robot.h>

#include <iostream>

const double CONTROLLER_MANAGER_ALLOWED_SAMPLE_ERROR_MS = 1.0;

namespace webots_ros2_control {

#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4 && HARDWARE_INTERFACE_VERSION_MINOR >= 12
  class WebotsResourceManager : public hardware_interface::ResourceManager {
  public:
    WebotsResourceManager(webots_ros2_driver::WebotsNode *node) :
      hardware_interface::ResourceManager(node->get_node_clock_interface(), node->get_node_logging_interface()),
      mHardwareLoader("webots_ros2_control", "webots_ros2_control::Ros2ControlSystemInterface"),
      mLogger(node->get_logger().get_child("WebotsResourceManager")) {
      mNode = node;
    }

    WebotsResourceManager(const WebotsResourceManager &) = delete;

    bool load_and_initialize_components(const std::string &urdf, unsigned int update_rate) override {
      components_are_loaded_and_initialized_ = true;

      std::vector<hardware_interface::HardwareInfo> controlHardware;
      try {
        controlHardware = hardware_interface::parse_control_resources_from_urdf(urdf);
      } catch (const std::runtime_error &ex) {
        throw std::runtime_error("URDF cannot be parsed by a `ros2_control` component parser: " + std::string(ex.what()));
      }
      for (unsigned int i = 0; i < controlHardware.size(); i++) {
        const std::string pluginName = controlHardware[i].hardware_plugin_name;

        std::unique_ptr<webots_ros2_control::Ros2ControlSystemInterface> webotsSystem;
        try {
          webotsSystem = std::unique_ptr<webots_ros2_control::Ros2ControlSystemInterface>(
            mHardwareLoader.createUnmanagedInstance(pluginName));
        } catch (pluginlib::PluginlibException &ex) {
          RCLCPP_ERROR(mLogger, "The plugin failed to load for some reason. Error: %s\n", ex.what());
          continue;
        }

        webotsSystem->init(mNode, controlHardware[i]);
        import_component(std::move(webotsSystem), controlHardware[i]);
      }

      return components_are_loaded_and_initialized_;
    }

  private:
    webots_ros2_driver::WebotsNode *mNode;
    pluginlib::ClassLoader<webots_ros2_control::Ros2ControlSystemInterface> mHardwareLoader;
    rclcpp::Logger mLogger;
  };
#endif

  Ros2Control::Ros2Control() {
    mNode = NULL;
  }

  void Ros2Control::step() {
    const int nowMs = wb_robot_get_time() * 1000.0;
    const int periodMs = nowMs - mLastControlUpdateMs;
    if (periodMs >= mControlPeriodMs && mNode->get_clock()->now() != rclcpp::Time(0, 0, mNode->get_clock()->get_clock_type())) {
      const rclcpp::Duration dt = rclcpp::Duration::from_seconds(mControlPeriodMs / 1000.0);
      mControllerManager->read(mNode->get_clock()->now(), dt);

      try {
        mControllerManager->update(mNode->get_clock()->now(), dt);
      } catch (const std::exception &ex) {
        RCLCPP_WARN_STREAM(mNode->get_logger(), "Controller manager update failed: " << ex.what());
      }

      mLastControlUpdateMs = nowMs;

      mControllerManager->write(mNode->get_clock()->now(), dt);
    }
  }
  void Ros2Control::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &) {
    mNode = node;
    mLastControlUpdateMs = 0;

    // Load hardware
    try {
      mHardwareLoader.reset(new pluginlib::ClassLoader<webots_ros2_control::Ros2ControlSystemInterface>(
        "webots_ros2_control", "webots_ros2_control::Ros2ControlSystemInterface"));
    } catch (pluginlib::LibraryLoadException &ex) {
      throw std::runtime_error("Hardware loader cannot be created: " + std::string(ex.what()));
    }

    // Control Hardware
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4 && HARDWARE_INTERFACE_VERSION_MINOR >= 12
    std::unique_ptr<hardware_interface::ResourceManager> resourceManager =
      std::make_unique<webots_ros2_control::WebotsResourceManager>(node);
#else
    std::string urdfString;
    std::vector<hardware_interface::HardwareInfo> controlHardware;
    std::unique_ptr<hardware_interface::ResourceManager> resourceManager =
      std::make_unique<hardware_interface::ResourceManager>();
    try {
      urdfString = mNode->urdf();
      controlHardware = hardware_interface::parse_control_resources_from_urdf(urdfString);
    } catch (const std::runtime_error &ex) {
      throw std::runtime_error("URDF cannot be parsed by a `ros2_control` component parser: " + std::string(ex.what()));
    }
    for (unsigned int i = 0; i < controlHardware.size(); i++) {
// Necessary hotfix for renamed variables present in "hardware_interface" package for versions above 3.5 (#590)
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4 || HARDWARE_INTERFACE_VERSION_MAJOR >= 3 && HARDWARE_INTERFACE_VERSION_MINOR >= 5
      const std::string pluginName = controlHardware[i].hardware_plugin_name;
      auto webotsSystem =
        std::unique_ptr<webots_ros2_control::Ros2ControlSystemInterface>(mHardwareLoader->createUnmanagedInstance(pluginName));
#else
      const std::string hardwareType = controlHardware[i].hardware_class_type;
      auto webotsSystem = std::unique_ptr<webots_ros2_control::Ros2ControlSystemInterface>(
        mHardwareLoader->createUnmanagedInstance(hardwareType));
#endif
      webotsSystem->init(mNode, controlHardware[i]);
      resourceManager->import_component(std::move(webotsSystem), controlHardware[i]);

      // Configure and activate all components
      using lifecycle_msgs::msg::State;
      rclcpp_lifecycle::State active_state(State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);
      resourceManager->set_component_state(controlHardware[i].name, active_state);

      resourceManager->load_urdf(urdfString, false, false);
    }
#endif

    // Controller Manager
    mExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
    options.arguments(node->get_node_options().arguments());
    mControllerManager.reset(new controller_manager::ControllerManager(std::move(resourceManager), mExecutor,
                                                                       "controller_manager", node->get_namespace(), options));

    // Update rate
    const int updateRate = mControllerManager->get_parameter("update_rate").as_int();
    mControlPeriodMs = (1.0 / updateRate) * 1000.0;

    int controlPeriodProductMs = wb_robot_get_basic_time_step();
    while (controlPeriodProductMs < mControlPeriodMs)
      controlPeriodProductMs += wb_robot_get_basic_time_step();

    if (abs(controlPeriodProductMs - mControlPeriodMs) > CONTROLLER_MANAGER_ALLOWED_SAMPLE_ERROR_MS)
      RCLCPP_WARN_STREAM(node->get_logger(), "Desired controller update period ("
                                               << mControlPeriodMs << "ms / " << updateRate
                                               << "Hz) is different from the Webots timestep ("
                                               << wb_robot_get_basic_time_step()
                                               << "ms). Please adjust the `update_rate` parameter in the `controller_manager` "
                                                  "or the `basicTimeStep` parameter in the Webots `WorldInfo` node.");

    // Spin
    mExecutor->add_node(mControllerManager);
    auto spin = [this]() {
      while (rclcpp::ok())
        mExecutor->spin_once();
    };
    mThreadExecutor = std::thread(spin);
  }
}  // namespace webots_ros2_control

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2Control, webots_ros2_driver::PluginInterface)
