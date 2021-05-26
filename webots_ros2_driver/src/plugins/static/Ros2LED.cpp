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

#include <webots_ros2_driver/plugins/static/Ros2LED.hpp>

#include <webots_ros2_driver/utils/Math.hpp>

namespace webots_ros2_driver
{
  void Ros2LED::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    mNode = node;
    mLED = mNode->robot()->getLED(parameters["name"]);

    assert(mLED != NULL);

    const std::string topicName = parameters.count("topicName") ? parameters["topicName"] : "~/" + getFixedNameString(parameters["name"]);
    mSubscriber = mNode->create_subscription<std_msgs::msg::ColorRGBA>(topicName, rclcpp::SensorDataQoS().reliable(), std::bind(&Ros2LED::onMessageReceived, this, std::placeholders::_1));
  }

  void Ros2LED::step()
  {
  }

  void Ros2LED::onMessageReceived(const std_msgs::msg::ColorRGBA::SharedPtr message)
  {
    const int red = message->r * 255;
    const int green = message->g * 255;
    const int blue = message->b * 255;
    mLED->set((red << 16) | (green << 8) | blue);
  }
}
