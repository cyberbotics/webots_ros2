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

#ifndef ROS2_VACUUM_GRIPPER_HPP
#define ROS2_VACUUM_GRIPPER_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <std_msgs/msg/bool.hpp>
#include <webots_ros2_msgs/msg/bool_stamped.hpp>
#include <webots_ros2_msgs/srv/get_bool.hpp>

namespace webots_ros2_driver {
  class Ros2VacuumGripper : public Ros2SensorPlugin {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    void publishPresence();

    // ROS2 subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mTurnOnSubscription;
    void turnOnCallback(const std_msgs::msg::Bool::SharedPtr message);

    // ROS2 services
    rclcpp::Service<webots_ros2_msgs::srv::GetBool>::SharedPtr mIsOnService;
    void isOnCallback(const std::shared_ptr<webots_ros2_msgs::srv::GetBool::Request> request,
                      std::shared_ptr<webots_ros2_msgs::srv::GetBool::Response> response);

    // ROS2 topics
    rclcpp::Publisher<webots_ros2_msgs::msg::BoolStamped>::SharedPtr mPresencePublisher;
    webots_ros2_msgs::msg::BoolStamped mPresenceMessage;
    bool mIsPresenceEnabled;

    // Device
    WbDeviceTag mVacuumGripper;
    // Runtime vars
    std::string mDeviceName;
  };

}  // namespace webots_ros2_driver

#endif
