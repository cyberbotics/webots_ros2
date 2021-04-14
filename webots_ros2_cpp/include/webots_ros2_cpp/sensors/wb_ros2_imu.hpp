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
#include <webots/InertialUnit.hpp>

// ros2
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace wb_ros2_interface {
namespace sensors {

class WbRos2Imu : public WbRos2Sensor {
public:
  WbRos2Imu(webots::InertialUnit* imu, const std::shared_ptr<rclcpp::Node> node);
  virtual ~WbRos2Imu();

  virtual void enable(int sampling_period) override { 
    imu_->enable(sampling_period); 
  };
  virtual void disable() override{ 
    imu_->disable(); 
  };

  void publish() override;


private:
  void pubImu();
  
  std::shared_ptr<webots::InertialUnit> imu_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

} // end namespace sensors
} // end namespace wb_ros2_interface
