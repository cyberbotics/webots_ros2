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

#ifndef ROS2_LIDAR_HPP
#define ROS2_LIDAR_HPP

#include <unordered_map>
#include <webots/Lidar.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <tf2_ros/static_transform_broadcaster.h>


namespace webots_ros2_driver
{
  class Ros2Lidar : public Ros2SensorPlugin
  {
  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

  private:
    void publishPointCloud();
    void publishLaserScan();

    webots::Lidar *mLidar;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mLaserPublisher;
    sensor_msgs::msg::LaserScan mLaserMessage;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mTfBroadcaster;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPublisher;
    sensor_msgs::msg::PointCloud2 mPointCloudMessage;

    bool mIsSensorEnabled;
    bool mIsPointCloudEnabled;
  };

} // end namespace webots_ros2_driver

#endif
