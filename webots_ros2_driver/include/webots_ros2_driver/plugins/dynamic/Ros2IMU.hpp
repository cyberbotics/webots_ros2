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

#ifndef ROS2_IMU_HPP
#define ROS2_IMU_HPP

#include <unordered_map>

#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>

#include <sensor_msgs/msg/imu.hpp>

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

namespace webots_ros2_driver {

  class Ros2IMU : public Ros2SensorPlugin {
  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

  private:
    void publishData();
    void enable();
    void disable();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mPublisher;
    sensor_msgs::msg::Imu mMessage;

    WbDeviceTag mInertialUnit;
    WbDeviceTag mGyro;
    WbDeviceTag mAccelerometer;

    bool mIsEnabled;
  };

}  // namespace webots_ros2_driver

#endif
