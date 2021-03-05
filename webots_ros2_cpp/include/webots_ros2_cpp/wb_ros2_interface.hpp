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
#include <webots_ros2_cpp/sensors/wb_ros2_lidar.hpp>

// webots
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// std
#include <unistd.h>
#include <memory>
#include <chrono>

namespace wb_ros2_interface {

class WbRos2Interface : public rclcpp::Node {
public:
  WbRos2Interface();
  virtual ~WbRos2Interface();

  void timerCallback();
  std::string name() const { return robot_name_; }
  static std::string fixedNameString(const std::string &name);
  virtual void setup();
  int getStep() { return step_; };

protected:
  virtual void setupRobot();
  virtual void setupSensors();
  virtual int step(int duration) { return robot_->step(duration); }

private:
  void fixName();
  std::string robot_name_;
  std::vector<std::shared_ptr<WbRos2Sensor>> sensors_;
  rclcpp::TimerBase::SharedPtr timer_;
  int step_;
  std::unique_ptr<webots::Supervisor> robot_;
};

} // end namespace wb_ros2_interface
