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

#include <webots_ros2_driver/plugins/static/Ros2LED.hpp>

#include <webots_ros2_driver/utils/Math.hpp>

#include <webots/robot.h>

namespace webots_ros2_driver {
  void Ros2LED::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    mNode = node;
    mLED = wb_robot_get_device(parameters["name"].c_str());

    assert(mLED != 0);

    const std::string topicName =
      parameters.count("topicName") ? parameters["topicName"] : "~/" + getFixedNameString(parameters["name"]);
    mSubscriber = mNode->create_subscription<std_msgs::msg::Int32>(
      topicName, rclcpp::SensorDataQoS().reliable(), std::bind(&Ros2LED::onMessageReceived, this, std::placeholders::_1));
  }

  void Ros2LED::step() {
  }

  void Ros2LED::onMessageReceived(const std_msgs::msg::Int32::SharedPtr message) {
    wb_led_set(mLED, message->data);
  }
}  // namespace webots_ros2_driver
