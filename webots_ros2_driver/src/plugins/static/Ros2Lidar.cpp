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

#include <webots_ros2_driver/plugins/static/Ros2Lidar.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <webots/robot.h>

namespace webots_ros2_driver {
  void Ros2Lidar::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    mLidar = wb_robot_get_device(parameters["name"].c_str());
    assert(mLidar != 0);
    if (parameters.count("updateRate") == 0)
      parameters.insert({"updateRate", std::to_string(wb_lidar_get_frequency(mLidar))});

    Ros2SensorPlugin::init(node, parameters);
    mIsSensorEnabled = false;
    mIsPointCloudEnabled = false;

    // Laser publisher
    if (wb_lidar_get_number_of_layers(mLidar) == 1) {
      mLaserPublisher = mNode->create_publisher<sensor_msgs::msg::LaserScan>(mTopicName, rclcpp::SensorDataQoS().reliable());
      const int resolution = wb_lidar_get_horizontal_resolution(mLidar);
      mLaserMessage.header.frame_id = mFrameName;
      mLaserMessage.angle_increment = -wb_lidar_get_fov(mLidar) / (resolution - 1);
      mLaserMessage.angle_min = wb_lidar_get_fov(mLidar) / 2.0;
      mLaserMessage.angle_max = -wb_lidar_get_fov(mLidar) / 2.0;
      mLaserMessage.time_increment = (double)wb_lidar_get_sampling_period(mLidar) / (1000.0 * resolution);
      mLaserMessage.scan_time = (double)wb_lidar_get_sampling_period(mLidar) / 1000.0;
      mLaserMessage.range_min = wb_lidar_get_min_range(mLidar);
      mLaserMessage.range_max = wb_lidar_get_max_range(mLidar);
      mLaserMessage.ranges.resize(resolution);
    }

    // Point cloud publisher
    mPointCloudPublisher =
      mNode->create_publisher<sensor_msgs::msg::PointCloud2>(mTopicName + "/point_cloud", rclcpp::SensorDataQoS().reliable());
    mPointCloudMessage.header.frame_id = mFrameName;
    mPointCloudMessage.height = 1;
    mPointCloudMessage.point_step = 20;
    mPointCloudMessage.is_dense = false;
    mPointCloudMessage.fields.resize(3);
    mPointCloudMessage.fields[0].name = "x";
    mPointCloudMessage.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mPointCloudMessage.fields[0].count = 1;
    mPointCloudMessage.fields[0].offset = 0;
    mPointCloudMessage.fields[1].name = "y";
    mPointCloudMessage.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mPointCloudMessage.fields[1].count = 1;
    mPointCloudMessage.fields[1].offset = 4;
    mPointCloudMessage.fields[2].name = "z";
    mPointCloudMessage.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mPointCloudMessage.fields[2].count = 1;
    mPointCloudMessage.fields[2].offset = 8;
    mPointCloudMessage.is_bigendian = false;

    if (mAlwaysOn) {
      wb_lidar_enable(mLidar, mPublishTimestepSyncedMs);
      wb_lidar_enable_point_cloud(mLidar);
      mIsSensorEnabled = true;
      mIsPointCloudEnabled = true;
    }
  }

  void Ros2Lidar::step() {
    if (!preStep())
      return;

    if (mIsSensorEnabled && mLaserPublisher != nullptr)
      publishLaserScan();

    if (mIsPointCloudEnabled)
      publishPointCloud();

    if (mAlwaysOn)
      return;

    const bool shouldPointCloudBeEnabled = mPointCloudPublisher->get_subscription_count() > 0;
    const bool shouldSensorBeEnabled =
      shouldPointCloudBeEnabled || (mLaserPublisher != nullptr && mLaserPublisher->get_subscription_count() > 0);

    // Enable/Disable sensor
    if (shouldSensorBeEnabled != mIsSensorEnabled) {
      if (shouldSensorBeEnabled)
        wb_lidar_enable(mLidar, mPublishTimestepSyncedMs);
      else
        wb_lidar_disable(mLidar);
      mIsSensorEnabled = shouldSensorBeEnabled;
    }

    // Enable/Disable point cloud
    if (shouldPointCloudBeEnabled != mIsPointCloudEnabled) {
      if (shouldPointCloudBeEnabled)
        wb_lidar_enable_point_cloud(mLidar);
      else
        wb_lidar_disable_point_cloud(mLidar);
      mIsPointCloudEnabled = shouldPointCloudBeEnabled;
    }
  }

  void Ros2Lidar::publishPointCloud() {
    auto data = wb_lidar_get_point_cloud(mLidar);
    if (data) {
      mPointCloudMessage.header.stamp = mNode->get_clock()->now();

      mPointCloudMessage.width = wb_lidar_get_number_of_points(mLidar);
      mPointCloudMessage.row_step = 20 * wb_lidar_get_number_of_points(mLidar);
      if (mPointCloudMessage.data.size() != mPointCloudMessage.row_step * mPointCloudMessage.height)
        mPointCloudMessage.data.resize(mPointCloudMessage.row_step * mPointCloudMessage.height);

      memcpy(mPointCloudMessage.data.data(), data, mPointCloudMessage.row_step * mPointCloudMessage.height);
      mPointCloudPublisher->publish(mPointCloudMessage);
    }
  }

  void Ros2Lidar::publishLaserScan() {
    auto rangeImage = wb_lidar_get_layer_range_image(mLidar, 0);
    if (rangeImage) {
      memcpy(mLaserMessage.ranges.data(), rangeImage, mLaserMessage.ranges.size() * sizeof(float));
      mLaserMessage.header.stamp = mNode->get_clock()->now();
      mLaserPublisher->publish(mLaserMessage);
    }
  }

}  // end namespace webots_ros2_driver
