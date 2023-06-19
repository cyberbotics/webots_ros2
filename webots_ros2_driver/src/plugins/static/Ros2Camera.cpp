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

#include <webots_ros2_driver/plugins/static/Ros2Camera.hpp>

#include <webots/robot.h>

namespace webots_ros2_driver {
  void Ros2Camera::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);
    mIsEnabled = false;
    mRecognitionIsEnabled = false;
    mCamera = wb_robot_get_device(parameters["name"].c_str());

    mCameraInfoSuffix = parameters.count("cameraInfoSuffix") ? parameters["cameraInfoSuffix"] : "/camera_info";
    mImageSuffix = parameters.count("imageSuffix") ? parameters["imageSuffix"] : "/image_color";

    assert(mCamera != 0);

    // Image publisher
    mImagePublisher =
      mNode->create_publisher<sensor_msgs::msg::Image>(mTopicName + mImageSuffix, rclcpp::SensorDataQoS().reliable());
    mImageMessage.header.frame_id = mFrameName;
    mImageMessage.height = wb_camera_get_height(mCamera);
    mImageMessage.width = wb_camera_get_width(mCamera);
    mImageMessage.is_bigendian = false;
    mImageMessage.step = sizeof(unsigned char) * 4 * wb_camera_get_width(mCamera);
    mImageMessage.data.resize(4 * wb_camera_get_width(mCamera) * wb_camera_get_height(mCamera));
    mImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;

    // CameraInfo publisher
    mCameraInfoPublisher =
      mNode->create_publisher<sensor_msgs::msg::CameraInfo>(mTopicName + mCameraInfoSuffix, rclcpp::SensorDataQoS().reliable());
    mCameraInfoMessage.header.stamp = mNode->get_clock()->now();
    mCameraInfoMessage.header.frame_id = mFrameName;
    mCameraInfoMessage.height = wb_camera_get_height(mCamera);
    mCameraInfoMessage.width = wb_camera_get_width(mCamera);
    mCameraInfoMessage.distortion_model = "plumb_bob";

    // Convert FoV to focal length.
    const double focalLength = wb_camera_get_width(mCamera) / (2 * tan(wb_camera_get_fov(mCamera) / 2));

    mCameraInfoMessage.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    mCameraInfoMessage.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    mCameraInfoMessage.k = {focalLength, 0.0,         (double)wb_camera_get_width(mCamera) / 2,
                            0.0,         focalLength, (double)wb_camera_get_height(mCamera) / 2,
                            0.0,         0.0,         1.0};
    mCameraInfoMessage.p = {focalLength,
                            0.0,
                            (double)wb_camera_get_width(mCamera) / 2,
                            0.0,
                            0.0,
                            focalLength,
                            (double)wb_camera_get_height(mCamera) / 2,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0};

    // Recognition publisher
    if (wb_camera_has_recognition(mCamera)) {
      mRecognitionPublisher = mNode->create_publisher<vision_msgs::msg::Detection2DArray>(mTopicName + "/recognitions",
                                                                                          rclcpp::SensorDataQoS().reliable());
      mWebotsRecognitionPublisher = mNode->create_publisher<webots_ros2_msgs::msg::CameraRecognitionObjects>(
        mTopicName + "/recognitions/webots", rclcpp::SensorDataQoS().reliable());
      mRecognitionMessage.header.frame_id = mFrameName;
      mWebotsRecognitionMessage.header.frame_id = mFrameName;
    }
  }

  void Ros2Camera::step() {
    if (!preStep())
      return;

    // Enable/Disable sensor
    const bool imageSubscriptionsExist = mImagePublisher->get_subscription_count() > 0;
    const bool recognitionSubscriptionsExist =
      (mRecognitionPublisher != nullptr && mRecognitionPublisher->get_subscription_count() > 0) ||
      (mWebotsRecognitionPublisher != nullptr && mWebotsRecognitionPublisher->get_subscription_count() > 0);
    const bool shouldBeEnabled = mAlwaysOn || imageSubscriptionsExist || recognitionSubscriptionsExist;

    if (shouldBeEnabled != mIsEnabled) {
      if (shouldBeEnabled)
        wb_camera_enable(mCamera, mPublishTimestepSyncedMs);
      else
        wb_camera_disable(mCamera);
      mIsEnabled = shouldBeEnabled;
    }

    if (recognitionSubscriptionsExist != mRecognitionIsEnabled) {
      if (recognitionSubscriptionsExist)
        wb_camera_recognition_enable(mCamera, mPublishTimestepSyncedMs);
      else
        wb_camera_recognition_disable(mCamera);
      mRecognitionIsEnabled = recognitionSubscriptionsExist;
    }

    // Publish data
    if (mAlwaysOn || imageSubscriptionsExist)
      publishImage();
    if (recognitionSubscriptionsExist)
      publishRecognition();
    if (mCameraInfoPublisher->get_subscription_count() > 0)
      mCameraInfoPublisher->publish(mCameraInfoMessage);
  }

  void Ros2Camera::publishImage() {
    auto image = wb_camera_get_image(mCamera);
    if (image) {
      mImageMessage.header.stamp = mNode->get_clock()->now();
      mCameraInfoMessage.header.stamp = mImageMessage.header.stamp;
      memcpy(mImageMessage.data.data(), image, mImageMessage.data.size());
      mImagePublisher->publish(mImageMessage);
    }
  }

  void Ros2Camera::publishRecognition() {
    if (wb_camera_recognition_get_number_of_objects(mCamera) == 0)
      return;

    auto objects = wb_camera_recognition_get_objects(mCamera);
    mRecognitionMessage.header.stamp = mNode->get_clock()->now();
    mWebotsRecognitionMessage.header.stamp = mNode->get_clock()->now();
    mRecognitionMessage.detections.clear();
    mWebotsRecognitionMessage.objects.clear();

    for (size_t i = 0; i < wb_camera_recognition_get_number_of_objects(mCamera); i++) {
      // Getting Object Info
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = objects[i].position[0];
      pose.pose.position.y = objects[i].position[1];
      pose.pose.position.z = objects[i].position[2];
      axisAngleToQuaternion(objects[i].orientation, pose.pose.orientation);

      // Transform to ROS camera coordinate frame
      // rpy = (0, pi/2, -pi/2)
      geometry_msgs::msg::TransformStamped transform;
      transform.transform.rotation.x = 0.5;
      transform.transform.rotation.y = -0.5;
      transform.transform.rotation.z = 0.5;
      transform.transform.rotation.w = 0.5;
      tf2::doTransform(pose, pose, transform);

      // Object Info -> Detection2D
      vision_msgs::msg::Detection2D detection;
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.pose.pose = pose.pose;
      detection.results.push_back(hypothesis);
      detection.bbox.center.position.x = objects[i].position_on_image[0];
      detection.bbox.center.position.y = objects[i].position_on_image[1];
      detection.bbox.size_x = objects[i].size_on_image[0];
      detection.bbox.size_y = objects[i].size_on_image[1];
      mRecognitionMessage.detections.push_back(detection);

      // Object Info -> CameraRecognitionObject
      webots_ros2_msgs::msg::CameraRecognitionObject recognitionWebotsObject;
      recognitionWebotsObject.id = objects[i].id;
      recognitionWebotsObject.model = std::string(objects[i].model);
      recognitionWebotsObject.pose = pose;
      recognitionWebotsObject.bbox.center.position.x = objects[i].position_on_image[0];
      recognitionWebotsObject.bbox.center.position.y = objects[i].position_on_image[1];
      recognitionWebotsObject.bbox.size_x = objects[i].size_on_image[0];
      recognitionWebotsObject.bbox.size_y = objects[i].size_on_image[1];
      for (size_t j = 0; j < objects[i].number_of_colors; j++) {
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
}  // namespace webots_ros2_driver
