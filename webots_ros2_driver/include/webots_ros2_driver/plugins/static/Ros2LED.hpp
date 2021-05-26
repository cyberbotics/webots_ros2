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

#ifndef ROS2_LED_HPP
#define ROS2_LED_HPP

#include <unordered_map>
#include <std_msgs/msg/color_rgba.hpp>
#include <webots/LED.hpp>
#include <webots_ros2_driver/utils/Utils.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>

namespace webots_ros2_driver
{

  class Ros2LED : public PluginInterface
  {
  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

  private:
    void onMessageReceived(const std_msgs::msg::ColorRGBA::SharedPtr message);

    webots::LED *mLED;
    webots_ros2_driver::WebotsNode *mNode;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr mSubscriber;
  };

}

#endif
