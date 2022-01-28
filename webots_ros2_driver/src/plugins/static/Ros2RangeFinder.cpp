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

#include <webots_ros2_driver/plugins/static/Ros2RangeFinder.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace webots_ros2_driver
{
  void Ros2RangeFinder::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    Ros2SensorPlugin::init(node, parameters);
    mIsEnabled = false;
    mRangeFinder = mNode->robot()->getRangeFinder(parameters["name"]);

    assert(mRangeFinder != NULL);

    // Image publisher
    mImagePublisher = mNode->create_publisher<sensor_msgs::msg::Image>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mImageMessage.header.frame_id = mFrameName;
    mImageMessage.height = mRangeFinder->getHeight();
    mImageMessage.width = mRangeFinder->getWidth();
    mImageMessage.is_bigendian = false;
    mImageMessage.step = sizeof(unsigned char) * 4 * mRangeFinder->getWidth();
    mImageMessage.data.resize(4 * mRangeFinder->getWidth() * mRangeFinder->getHeight());
    mImageMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    // CameraInfo publisher
    rclcpp::QoS cameraInfoQos(1);
    cameraInfoQos.reliable();
    cameraInfoQos.transient_local();
    cameraInfoQos.keep_last(1);
    mCameraInfoPublisher = mNode->create_publisher<sensor_msgs::msg::CameraInfo>(mTopicName + "/camera_info", cameraInfoQos);
    mCameraInfoMessage.header.stamp = mNode->get_clock()->now();
    mCameraInfoMessage.header.frame_id = mFrameName;
    mCameraInfoMessage.height = mRangeFinder->getHeight();
    mCameraInfoMessage.width = mRangeFinder->getWidth();
    mCameraInfoMessage.distortion_model = "plumb_bob";
    const double focalLength = 0.5 * mRangeFinder->getWidth() * (1 / tan(0.5 * mRangeFinder->getFov()));
    mCameraInfoMessage.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    mCameraInfoMessage.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    mCameraInfoMessage.k = {
        focalLength, 0.0, (double)mRangeFinder->getWidth() / 2,
        0.0, focalLength, (double)mRangeFinder->getHeight() / 2,
        0.0, 0.0, 1.0};
    mCameraInfoMessage.p = {
        focalLength, 0.0, (double)mRangeFinder->getWidth() / 2, 0.0,
        0.0, focalLength, (double)mRangeFinder->getHeight() / 2, 0.0,
        0.0, 0.0, 1.0, 0.0};
    mCameraInfoPublisher->publish(mCameraInfoMessage);

    // Point cloud publisher
    mPointCloudPublisher = mNode->create_publisher<sensor_msgs::msg::PointCloud2>(mTopicName + "/point_cloud", rclcpp::SensorDataQoS().reliable());
    mPointCloudMessage.header.frame_id = mFrameName;
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
    const int width = mRangeFinder->getWidth();
    const int height = mRangeFinder->getHeight();
    mPointCloudMessage.width = width;
    mPointCloudMessage.height = height;
    mPointCloudMessage.point_step = 12;
    mPointCloudMessage.row_step = width * 12;
    mPointCloudMessage.data.resize(width * height * 12);

    if (mAlwaysOn) {
      mRangeFinder->enable(mPublishTimestepSyncedMs);
      mIsEnabled = true;
    }
  }

  void Ros2RangeFinder::step()
  {
    if (!preStep())
      return;

    if (mIsEnabled)
      publishImage();
    
    if (mIsEnabled)
      publishPointCloud();

    if (mAlwaysOn)
      return;

    // Enable/Disable sensor
    const bool shouldBeEnabled = mImagePublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mRangeFinder->enable(mPublishTimestepSyncedMs);
      else
        mRangeFinder->disable();
      mIsEnabled = shouldBeEnabled;
    }
  }

  void Ros2RangeFinder::publishImage()
  {
    auto image = mRangeFinder->getRangeImage();
    if (image)
    {
      mImageMessage.header.stamp = mNode->get_clock()->now();
      memcpy(mImageMessage.data.data(), image, mImageMessage.data.size());
      mImagePublisher->publish(mImageMessage);
    }
  }

  void Ros2RangeFinder::publishPointCloud()
  {
    auto image = mRangeFinder->getRangeImage();
    if (image)
    {
      mPointCloudMessage.header.stamp = mNode->get_clock()->now();

      const int width = mCameraInfoMessage.width;
      const int height = mCameraInfoMessage.height;
      const float cx = mCameraInfoMessage.k[2];
      const float cy = mCameraInfoMessage.k[5];
      const float fx = mCameraInfoMessage.k[0];
      const float fy = mCameraInfoMessage.k[4];

      float* data = (float*)mPointCloudMessage.data.data();
      for (int j = 0; j < height; j++)
      {
        for (int i = 0; i < width; i++)
        {
          int idx = i + j * width;
          float x = image[idx];
          float y = -(i - cx) * x / fx;
          float z = -(j - cy) * x / fy;
          memcpy(data + idx * 3    , &x, sizeof(float));
          memcpy(data + idx * 3 + 1, &y, sizeof(float));
          memcpy(data + idx * 3 + 2, &z, sizeof(float));
        }
      }
      mPointCloudPublisher->publish(mPointCloudMessage);
    }
  }
}