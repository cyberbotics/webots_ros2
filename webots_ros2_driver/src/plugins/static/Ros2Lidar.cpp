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

#include <webots_ros2_driver/plugins/static/Ros2Lidar.hpp>

#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace webots_ros2_driver
{
  void Ros2Lidar::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    Ros2SensorPlugin::init(node, parameters);
    mIsSensorEnabled = false;
    mIsPointCloudEnabled = false;
    mLidar = mNode->robot()->getLidar(parameters["name"]);

    assert(mLidar != NULL);

    // Laser publisher
    if (mLidar->getNumberOfLayers() == 1)
    {
      mLaserPublisher = mNode->create_publisher<sensor_msgs::msg::LaserScan>(mTopicName, rclcpp::SensorDataQoS().reliable());
      const int resolution = mLidar->getHorizontalResolution();
      mLaserMessage.header.frame_id = mFrameName + "_rotated";
      mLaserMessage.angle_increment = mLidar->getFov() / resolution;
      mLaserMessage.angle_min = -mLidar->getFov() / 2.0;
      mLaserMessage.angle_max = mLidar->getFov() / 2.0 - mLaserMessage.angle_increment;
      mLaserMessage.time_increment = (double)mLidar->getSamplingPeriod() / (1000.0 * resolution);
      mLaserMessage.scan_time = (double)mLidar->getSamplingPeriod() / 1000.0;
      mLaserMessage.range_min = mLidar->getMinRange();
      mLaserMessage.range_max = mLidar->getMaxRange();
      mLaserMessage.ranges.resize(resolution);

      mTfBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(mNode);
      auto transformStamped = geometry_msgs::msg::TransformStamped();
      transformStamped.header.frame_id = mFrameName;
      transformStamped.child_frame_id = mFrameName + "_rotated";
      transformStamped.transform.rotation.x = 0.5;
      transformStamped.transform.rotation.y = 0.5;
      transformStamped.transform.rotation.z = -0.5;
      transformStamped.transform.rotation.w = 0.5;
      mTfBroadcaster->sendTransform(transformStamped);
    }

    // Point cloud publisher
    mPointCloudPublisher = mNode->create_publisher<sensor_msgs::msg::PointCloud2>(mTopicName + "/point_cloud", rclcpp::SensorDataQoS().reliable());
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
      mLidar->enable(mPublishTimestepSyncedMs);
      mLidar->enablePointCloud();
      mIsSensorEnabled = true;
      mIsPointCloudEnabled = true;
    }
  }

  void Ros2Lidar::step()
  {
    if (!preStep())
      return;

    if (mIsSensorEnabled && mLaserPublisher != nullptr)
      publishLaserScan();

    if (mIsPointCloudEnabled)
      publishPointCloud();

    if (mAlwaysOn)
      return;
    
    const bool shouldPointCloudBeEnabled = mPointCloudPublisher->get_subscription_count() > 0;
    const bool shouldSensorBeEnabled = shouldPointCloudBeEnabled ||
                                 (mLaserPublisher != nullptr && mLaserPublisher->get_subscription_count() > 0);

    // Enable/Disable sensor
    if (shouldSensorBeEnabled != mIsSensorEnabled)
    {
      if (shouldSensorBeEnabled)
        mLidar->enable(mPublishTimestepSyncedMs);
      else
        mLidar->disable();
      mIsSensorEnabled = shouldSensorBeEnabled;
    }

    // Enable/Disable point cloud
    if (shouldPointCloudBeEnabled != mIsPointCloudEnabled)
    {
      if (shouldPointCloudBeEnabled)
        mLidar->enablePointCloud();
      else
        mLidar->disablePointCloud();
      mIsPointCloudEnabled = shouldPointCloudBeEnabled;
    }
  }

  void Ros2Lidar::publishPointCloud()
  {
    auto data = mLidar->getPointCloud();
    if (data)
    {
      mPointCloudMessage.header.stamp = mNode->get_clock()->now();

      mPointCloudMessage.width = mLidar->getNumberOfPoints();
      mPointCloudMessage.row_step = 20 * mLidar->getNumberOfPoints();
      if (mPointCloudMessage.data.size() != mPointCloudMessage.row_step * mPointCloudMessage.height)
        mPointCloudMessage.data.resize(mPointCloudMessage.row_step * mPointCloudMessage.height);

      memcpy(mPointCloudMessage.data.data(), data, mPointCloudMessage.row_step * mPointCloudMessage.height);
      mPointCloudPublisher->publish(mPointCloudMessage);
    }
  }

  void Ros2Lidar::publishLaserScan()
  {
    auto rangeImage = mLidar->getLayerRangeImage(0);
    if (rangeImage)
    {
      memcpy(mLaserMessage.ranges.data(), rangeImage, mLaserMessage.ranges.size() * sizeof(float));
      mLaserMessage.header.stamp = mNode->get_clock()->now();
      mLaserPublisher->publish(mLaserMessage);
    }
  }

} // end namespace webots_ros2_driver
