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

#include <webots_ros2_driver/plugins/static/Ros2Camera.hpp>

namespace webots_ros2_driver
{
  void Ros2Camera::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    Ros2SensorPlugin::init(node, parameters);
    mIsEnabled = false;
    mRecognitionIsEnabled = false;
    mCamera = mNode->robot()->getCamera(parameters["name"]);

    assert(mCamera != NULL);

    // Image publisher
    mImagePublisher = mNode->create_publisher<sensor_msgs::msg::Image>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mImageMessage.header.frame_id = mFrameName;
    mImageMessage.height = mCamera->getHeight();
    mImageMessage.width = mCamera->getWidth();
    mImageMessage.is_bigendian = false;
    mImageMessage.step = sizeof(unsigned char) * 4 * mCamera->getWidth();
    mImageMessage.data.resize(4 * mCamera->getWidth() * mCamera->getHeight());
    mImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;

    // CameraInfo publisher
    rclcpp::QoS cameraInfoQos(1);
    cameraInfoQos.reliable();
    cameraInfoQos.transient_local();
    cameraInfoQos.keep_last(1);
    mCameraInfoPublisher = mNode->create_publisher<sensor_msgs::msg::CameraInfo>(mTopicName + "/camera_info", cameraInfoQos);
    mCameraInfoMessage.header.stamp = mNode->get_clock()->now();
    mCameraInfoMessage.header.frame_id = mFrameName;
    mCameraInfoMessage.height = mCamera->getHeight();
    mCameraInfoMessage.width = mCamera->getWidth();
    mCameraInfoMessage.distortion_model = "plumb_bob";
    const double focalLength = (mCamera->getFocalLength() == 0) ? 570.34 : mCamera->getFocalLength();
    mCameraInfoMessage.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    mCameraInfoMessage.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    mCameraInfoMessage.k = {
        focalLength, 0.0, (double)mCamera->getWidth() / 2,
        0.0, focalLength, (double)mCamera->getHeight() / 2,
        0.0, 0.0, 1.0};
    mCameraInfoMessage.p = {
        focalLength, 0.0, (double)mCamera->getWidth() / 2, 0.0,
        0.0, focalLength, (double)mCamera->getHeight() / 2, 0.0,
        0.0, 0.0, 1.0, 0.0};
    mCameraInfoPublisher->publish(mCameraInfoMessage);

    // Recognition publisher
    if (mCamera->hasRecognition())
    {
      mRecognitionPublisher = mNode->create_publisher<vision_msgs::msg::Detection2DArray>(
          mTopicName + "/recognitions",
          rclcpp::SensorDataQoS().reliable());
      mWebotsRecognitionPublisher = mNode->create_publisher<
          webots_ros2_msgs::msg::WbCameraRecognitionObjects>(
          mTopicName + "/recognitions/webots",
          rclcpp::SensorDataQoS().reliable());
      mRecognitionMessage.header.frame_id = mFrameName;
      mWebotsRecognitionMessage.header.frame_id = mFrameName;
    }
  }

  void Ros2Camera::step()
  {
    if (!preStep())
      return;

    // Enable/Disable sensor
    const bool imageSubscriptionsExist = mImagePublisher->get_subscription_count() > 0;
    const bool recognitionSubscriptionsExist =
        (mRecognitionPublisher != nullptr && mRecognitionPublisher->get_subscription_count() > 0) ||
        (mWebotsRecognitionPublisher != nullptr && mWebotsRecognitionPublisher->get_subscription_count() > 0);
    const bool shouldBeEnabled = mAlwaysOn || imageSubscriptionsExist || recognitionSubscriptionsExist;

    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mCamera->enable(mPublishTimestepSyncedMs);
      else
        mCamera->disable();
      mIsEnabled = shouldBeEnabled;
    }

    if (recognitionSubscriptionsExist != mRecognitionIsEnabled)
    {
      if (recognitionSubscriptionsExist)
        mCamera->recognitionEnable(mPublishTimestepSyncedMs);
      else
        mCamera->recognitionDisable();
      mRecognitionIsEnabled = recognitionSubscriptionsExist;
    }

    // Publish data
    if (mAlwaysOn || imageSubscriptionsExist)
      publishImage();
    if (recognitionSubscriptionsExist)
      publishRecognition();
  }

  void Ros2Camera::publishImage()
  {
    auto image = mCamera->getImage();
    if (image)
    {
      mImageMessage.header.stamp = mNode->get_clock()->now();
      memcpy(mImageMessage.data.data(), image, mImageMessage.data.size());
      mImagePublisher->publish(mImageMessage);
    }
  }

  void Ros2Camera::publishRecognition()
  {
    if (mCamera->getRecognitionNumberOfObjects() == 0)
      return;

    auto objects = mCamera->getRecognitionObjects();
    mRecognitionMessage.header.stamp = mNode->get_clock()->now();
    mWebotsRecognitionMessage.header.stamp = mNode->get_clock()->now();
    mRecognitionMessage.detections.clear();
    mWebotsRecognitionMessage.objects.clear();

    for (size_t i = 0; i < mCamera->getRecognitionNumberOfObjects(); i++)
    {
      // Getting Object Info
      geometry_msgs::msg::Point position;
      geometry_msgs::msg::Quaternion orientation;
      position.x = objects[i].position[0];
      position.y = objects[i].position[1];
      position.z = objects[i].position[2];
      axisAngleToQuaternion(objects[i].orientation, orientation);

      // Object Info -> Detection2D
      vision_msgs::msg::Detection2D detection;
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.pose.pose.position = position;
      hypothesis.pose.pose.orientation = orientation;
      detection.results.push_back(hypothesis);
      detection.bbox.center.x = objects[i].position_on_image[0];
      detection.bbox.center.y = objects[i].position_on_image[1];
      detection.bbox.size_x = objects[i].size_on_image[0];
      detection.bbox.size_y = objects[i].size_on_image[1];
      mRecognitionMessage.detections.push_back(detection);

      // Object Info -> WbCameraRecognitionObject
      webots_ros2_msgs::msg::WbCameraRecognitionObject recognitionWebotsObject;
      recognitionWebotsObject.id = objects[i].id;
      recognitionWebotsObject.model = std::string(objects[i].model);
      recognitionWebotsObject.pose.pose.position = position;
      recognitionWebotsObject.pose.pose.orientation = orientation;
      recognitionWebotsObject.bbox.center.x = objects[i].position_on_image[0];
      recognitionWebotsObject.bbox.center.y = objects[i].position_on_image[1];
      recognitionWebotsObject.bbox.size_x = objects[i].size_on_image[0];
      recognitionWebotsObject.bbox.size_y = objects[i].size_on_image[1];
      for (size_t j = 0; j < objects[i].number_of_colors; j++)
      {
        std_msgs::msg::ColorRGBA color;
        color.r = objects[i].colors[3 * j];
        color.g = objects[i].colors[3 * j + 1];
        color.b = objects[i].colors[3 * j + 2];
        recognitionWebotsObject.colors.push_back(color);
      }
      mWebotsRecognitionMessage.objects.push_back(recognitionWebotsObject);
    }
    mWebotsRecognitionPublisher->publish(mWebotsRecognitionMessage);
    mRecognitionPublisher->publish(mRecognitionMessage);
  }
}
