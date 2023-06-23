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

#ifndef ROS2_PEN_HPP
#define ROS2_PEN_HPP

#include <unordered_map>

#include <webots/pen.h>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/utils/Utils.hpp>
#include <webots_ros2_msgs/msg/pen_ink_properties.hpp>

namespace webots_ros2_driver {

  class Ros2Pen : public PluginInterface {
  public:
    // Parameters should be const&.
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

  private:
    void onEnableMessage(const std_msgs::msg::Bool::SharedPtr message);
    void onColorMessage(const webots_ros2_msgs::msg::PenInkProperties::SharedPtr message);

    WbDeviceTag mDeviceTag;
    webots_ros2_driver::WebotsNode *mNode;  // This should really be a shared_ptr
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mEnableSubscription;
    rclcpp::Subscription<webots_ros2_msgs::msg::PenInkProperties>::SharedPtr mColorSubscription;
  };

}  // namespace webots_ros2_driver

#endif
