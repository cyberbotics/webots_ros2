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

#ifndef ROS2_CAMERA_HPP
#define ROS2_CAMERA_HPP

#include <unordered_map>

#include <webots/camera.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>
#include <webots_ros2_driver/utils/Math.hpp>
#include <webots_ros2_msgs/msg/camera_recognition_object.hpp>
#include <webots_ros2_msgs/msg/camera_recognition_objects.hpp>

namespace webots_ros2_driver {

  class Ros2Camera : public Ros2SensorPlugin {
  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

  private:
    void publishImage();
    void publishRecognition();

    WbDeviceTag mCamera;

    std::string mCameraInfoSuffix;
    std::string mImageSuffix;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePublisher;
    sensor_msgs::msg::Image mImageMessage;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mCameraInfoPublisher;
    sensor_msgs::msg::CameraInfo mCameraInfoMessage;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr mRecognitionPublisher;
    rclcpp::Publisher<webots_ros2_msgs::msg::CameraRecognitionObjects>::SharedPtr mWebotsRecognitionPublisher;
    vision_msgs::msg::Detection2DArray mRecognitionMessage;
    webots_ros2_msgs::msg::CameraRecognitionObjects mWebotsRecognitionMessage;

    bool mIsEnabled;
    bool mRecognitionIsEnabled;
  };

}  // namespace webots_ros2_driver

#endif
