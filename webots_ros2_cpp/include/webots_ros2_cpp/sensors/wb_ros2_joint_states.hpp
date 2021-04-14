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

// wb_ros2_cpp
#include <webots_ros2_cpp/wb_ros2_sensor.hpp>

// webots
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>

// ros2
#include <sensor_msgs/msg/joint_state.hpp>

namespace wb_ros2_interface {
namespace sensors {

class WbRos2JointState : public WbRos2Sensor {
public:
  WbRos2JointState(const std::shared_ptr<rclcpp::Node> node);
  virtual ~WbRos2JointState();

  virtual void enable(int sampling_period) override;
  virtual void disable() override;

  void addPositionSensor(webots::PositionSensor* position_sensor, 
    int sampling_period);
  void publish() override;


private:
  void pubJoints();
  
  std::vector<std::shared_ptr<webots::PositionSensor>> position_sensors_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  sensor_msgs::msg::JointState joint_msg_;
};

} // end namespace sensors
} // end namespace wb_ros2_interface
