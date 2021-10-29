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
    const double focalLength = 570.34;
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
}
