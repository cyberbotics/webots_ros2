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

#pragma once

// webots_ros2_cpp
#include <webots_ros2_cpp/wb_ros2_device.hpp>
#include <webots_ros2_cpp/wb_ros2_sensor.hpp>
#include <webots_ros2_cpp/sensors/wb_ros2_camera.hpp>
#include <webots_ros2_cpp/sensors/wb_ros2_gps.hpp>
#include <webots_ros2_cpp/sensors/wb_ros2_imu.hpp>
#include <webots_ros2_cpp/sensors/wb_ros2_joint_states.hpp>
#include <webots_ros2_cpp/sensors/wb_ros2_lidar.hpp>

// webots
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// std
#include <memory>
#include <chrono>

namespace webots_ros2
{

  class WebotsNode : public rclcpp::Node
  {
  public:
    WebotsNode();
    void registerPlugin(const std::string &pathToPlugin, const std::map<std::string, std::string> &arguments);
    void init();

  private:
    void timerCallback();
    static std::string fixedNameString(const std::string &name);

    std::string mRobotName;
    rclcpp::TimerBase::SharedPtr mTimer;
    int mStep;
    std::shared_ptr<webots::Supervisor> mRobot;
  };

} // end namespace webots_ros2
