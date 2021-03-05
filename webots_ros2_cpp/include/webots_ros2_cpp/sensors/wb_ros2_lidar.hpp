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

// wb_ros2_interface
#include <webots_ros2_cpp/wb_ros2_sensor.hpp>

// webots
#include <webots/Lidar.hpp>

// ros2
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>

namespace wb_ros2_interface {
namespace sensors {

class WbRos2Lidar : public WbRos2Sensor {
public:
  WbRos2Lidar(webots::Lidar* lidar, const std::shared_ptr<rclcpp::Node> node);
  virtual ~WbRos2Lidar();

  virtual void enable(int sampling_period) override { 
    lidar_->enable(sampling_period); 
  };
  virtual void disable() override{ 
    lidar_->disable(); 
  };

  void publish() override;


private:
  void pubPointCloud();
  void pubLaserScan();
  
  std::shared_ptr<webots::Lidar> lidar_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub_;
};

} // end namespace sensors
} // end namespace wb_ros2_interface
