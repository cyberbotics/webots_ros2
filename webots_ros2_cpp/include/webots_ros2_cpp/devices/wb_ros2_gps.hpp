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
#include <webots/GPS.hpp>

// ros2
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>

namespace wb_ros2_interface {
namespace sensors {

class WbRos2GPS : public WbRos2Sensor {
public:
  WbRos2GPS(webots::GPS* gps, const std::shared_ptr<rclcpp::Node> node);
  virtual ~WbRos2GPS();

  virtual void enable(int sampling_period) override { 
    gps_->enable(sampling_period); 
  };
  virtual void disable() override{ 
    gps_->disable(); 
  };

  void publish() override;


private:
  void pubGPSPointStamped();
  void pubGPSNavSat();
  
  std::shared_ptr<webots::GPS> gps_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gps_pub2_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
  bool wsg84;
};

} // end namespace sensors
} // end namespace wb_ros2_interface
