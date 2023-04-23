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

#include <webots_ros2_driver/plugins/dynamic/Ros2RGBD.hpp>

#include <webots_ros2_driver/utils/Math.hpp>
#include "pluginlib/class_list_macros.hpp"

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

namespace webots_ros2_driver {
  void Ros2RGBD::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);
    mCamera = 0;
    mRangeFinder = 0;

    if (!parameters.count("camera") || !parameters.count("rangeFinder"))
      throw std::runtime_error("The RGBD plugin has to contain <camera> and <rangeFinder>");

    mCamera = wb_robot_get_device(parameters["camera"].c_str());
    if (mCamera == 0 || wb_device_get_node_type(mCamera) != WB_NODE_CAMERA)
      throw std::runtime_error("Cannot find Webots camera `" + parameters["camera"] + "` for the RGBD plugin.");

    mRangeFinder = wb_robot_get_device(parameters["rangeFinder"].c_str());
    if (mRangeFinder == 0 || wb_device_get_node_type(mRangeFinder) != WB_NODE_RANGE_FINDER)
      throw std::runtime_error("Cannot find Webots range finder `" + parameters["rangeFinder"] + "` for the RGBD plugin.");

    // Verify resolution
    const int width = wb_camera_get_width(mCamera);
    const int height = wb_camera_get_height(mCamera);
    const int range_finder_width = wb_range_finder_get_width(mRangeFinder);
    const int range_finder_height = wb_range_finder_get_height(mRangeFinder);
    if (width != range_finder_width || height != range_finder_height)
      throw std::runtime_error("The width and height of the camera and range finder must be the same for the RGBD plugin.");

    // Verify field of view
    const double field_of_view = wb_camera_get_fov(mCamera);
    const double range_finder_field_of_view = wb_range_finder_get_fov(mRangeFinder);
    if (field_of_view != range_finder_field_of_view)
      throw std::runtime_error("The field of view of the camera and range finder must be the same for the RGBD plugin.");

    // Camera params
    mWidth = width;
    mHeight = height;
    mFocalLengthX = 0.5 * width * (1 / tan(0.5 * range_finder_field_of_view));
    mFocalLengthY = 0.5 * height * (1 / tan(0.5 * range_finder_field_of_view));
    mCenterX = 0.5 * width;
    mCenterY = 0.5 * height;

    // Prepare ROS2 message
    mPublisher = mNode->create_publisher<sensor_msgs::msg::PointCloud2>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mMessage.header.frame_id = mFrameName;
    mMessage.fields.resize(4);
    mMessage.fields[0].name = "x";
    mMessage.fields[0].offset = 0;
    mMessage.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mMessage.fields[0].count = 1;
    mMessage.fields[1].name = "y";
    mMessage.fields[1].offset = 4;
    mMessage.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mMessage.fields[1].count = 1;
    mMessage.fields[2].name = "z";
    mMessage.fields[2].offset = 8;
    mMessage.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mMessage.fields[2].count = 1;
    mMessage.fields[3].name = "rgb";
    mMessage.fields[3].offset = 16;
    mMessage.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    mMessage.fields[3].count = 1;
    mMessage.is_bigendian = false;
    mMessage.width = width;
    mMessage.height = height;
    mMessage.point_step = 20;
    mMessage.row_step = width * 20;
    mMessage.data.resize(width * 20 * height);
  }

  void Ros2RGBD::step() {
    if (!preStep())
      return;

    if (mPublisher->get_subscription_count() > 0) {
      wb_camera_enable(mCamera, mPublishTimestepSyncedMs);
      wb_range_finder_enable(mRangeFinder, mPublishTimestepSyncedMs);
      publishData();
    }
  }

  void Ros2RGBD::publishData() {
    const float *depth_image = wb_range_finder_get_range_image(mRangeFinder);
    const unsigned char *rgb_image = wb_camera_get_image(mCamera);
    if (depth_image && rgb_image) {
      mMessage.header.stamp = mNode->get_clock()->now();

      int idx;
      float x, y, z;
      int rgb;

      float *data = (float *)mMessage.data.data();
      for (int j = 0; j < mHeight; j++) {
        for (int i = 0; i < mWidth; i++) {
          idx = i + j * mWidth;
          x = depth_image[idx];
          y = -(i - mCenterX) * x / mFocalLengthX;
          z = -(j - mCenterY) * x / mFocalLengthY;

          memcpy(data + idx * 5, &x, sizeof(float));
          memcpy(data + idx * 5 + 1, &y, sizeof(float));
          memcpy(data + idx * 5 + 2, &z, sizeof(float));

          rgb = wb_camera_image_get_red(rgb_image, mWidth, i, j) << 16;
          rgb |= wb_camera_image_get_green(rgb_image, mWidth, i, j) << 8 | wb_camera_image_get_blue(rgb_image, mWidth, i, j);
          memcpy(data + idx * 5 + 4, &rgb, sizeof(float));
        }
      }
      mPublisher->publish(mMessage);
    }
  }
}  // namespace webots_ros2_driver

PLUGINLIB_EXPORT_CLASS(webots_ros2_driver::Ros2RGBD, webots_ros2_driver::PluginInterface)
