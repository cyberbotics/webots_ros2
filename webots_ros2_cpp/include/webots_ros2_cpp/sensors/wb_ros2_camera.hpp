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

// wb_ros2
#include <webots_ros2_cpp/wb_ros2_sensor.hpp>
#include <webots_ros2_msgs/msg/wb_camera_recognition_object.hpp>
#include <webots_ros2_msgs/msg/wb_camera_recognition_objects.hpp>
#include <webots_ros2_cpp/math.hpp>

// webots
#include <webots/Camera.hpp>

// ros2
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace wb_ros2_interface {
namespace sensors {

class WbRos2Camera : public WbRos2Sensor {
public:
  WbRos2Camera(webots::Camera* camera, const std::shared_ptr<rclcpp::Node> node);
  virtual ~WbRos2Camera();

  virtual void enable(int sampling_period) override;
  virtual void disable() override{ 
    camera_->disable(); 
  };

  void publish() override;

private:
  void pubImage();
  void pubRecognition();
  void createCameraInfoMsg();
  
  std::shared_ptr<webots::Camera> camera_;
  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr recog_pub_;
  rclcpp::Publisher<webots_ros2_msgs::msg::WbCameraRecognitionObjects>::SharedPtr 
    wb_recog_pub_;
};

} // end namespace sensors
} // end namespace wb_ros2_interface
