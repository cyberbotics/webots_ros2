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

// webots
#include <webots/Device.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>

namespace wb_ros2_interface {

class WbRos2Sensor {
public:
  virtual ~WbRos2Sensor(){};
  WbRos2Sensor(const std::shared_ptr<rclcpp::Node> node) :
    node_(node)
  {};
  
  virtual void publish() = 0;
  virtual void enable(int samplingPeriod) = 0;
  virtual void disable() = 0;

protected:
  std::weak_ptr<rclcpp::Node> node_;

};

} // end namespace wb_ros2_interface
