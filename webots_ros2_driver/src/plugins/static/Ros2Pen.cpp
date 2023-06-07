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

#include <webots_ros2_driver/plugins/static/Ros2Pen.hpp>

#include <webots_ros2_driver/utils/Math.hpp>

#include <webots/robot.h>

namespace webots_ros2_driver {
  void Ros2Pen::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    mNode = node;
    mDeviceTag = wb_robot_get_device(parameters["name"].c_str());

    if (mDeviceTag == 0) {
      RCLCPP_ERROR(node->get_logger(), "No device '%s' exists.", parameters["name"].c_str());
    }
    assert(mDeviceTag != 0);

    const std::string enableSuffix = "/write";
    const std::string colorSuffix = "/set_ink_color";
    const std::string topicName =
      parameters.count("topicName") ? parameters["topicName"] : "~/" + getFixedNameString(parameters["name"]);

    mEnableSubscription =
      mNode->create_subscription<std_msgs::msg::Bool>(topicName + enableSuffix, rclcpp::SensorDataQoS().reliable(),
                                                      std::bind(&Ros2Pen::onEnableMessage, this, std::placeholders::_1));
    mColorSubscription = mNode->create_subscription<webots_ros2_msgs::msg::PenInkProperties>(
      topicName + colorSuffix, rclcpp::SensorDataQoS().reliable(),
      std::bind(&Ros2Pen::onColorMessage, this, std::placeholders::_1));
  }

  void Ros2Pen::step() {
  }

  void Ros2Pen::onEnableMessage(const std_msgs::msg::Bool::SharedPtr message) {
    wb_pen_write(mDeviceTag, message->data);
  }

  void Ros2Pen::onColorMessage(const webots_ros2_msgs::msg::PenInkProperties::SharedPtr message) {
    wb_pen_set_ink_color(mDeviceTag, message->color, message->density);
  }
}  // namespace webots_ros2_driver
