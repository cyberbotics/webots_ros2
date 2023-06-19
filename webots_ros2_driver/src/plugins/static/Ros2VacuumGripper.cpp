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

#include "webots_ros2_driver/plugins/static/Ros2VacuumGripper.hpp"

#include <webots/robot.h>
#include <webots/vacuum_gripper.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace webots_ros2_driver {
  void Ros2VacuumGripper::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);

    // This parameter is read when loading the URDF file
    mDeviceName = parameters.count("name") ? parameters["name"] : "vacuum gripper";
    mVacuumGripper = wb_robot_get_device(mDeviceName.c_str());

    assert(mVacuumGripper != 0);

    // Initialize services, publishers and subcriptions
    mTurnOnSubscription = mNode->create_subscription<std_msgs::msg::Bool>(
      mTopicName + "/turn_on", rclcpp::SensorDataQoS().reliable(), std::bind(&Ros2VacuumGripper::turnOnCallback, this, _1));

    mIsOnService = mNode->create_service<webots_ros2_msgs::srv::GetBool>(
      mTopicName + "/is_on", std::bind(&Ros2VacuumGripper::isOnCallback, this, _1, _2));

    mIsPresenceEnabled = false;
    mPresencePublisher =
      mNode->create_publisher<webots_ros2_msgs::msg::BoolStamped>(mTopicName + "/presence", rclcpp::SensorDataQoS().reliable());
  }

  void Ros2VacuumGripper::turnOnCallback(const std_msgs::msg::Bool::SharedPtr message) {
    if (message->data)
      wb_vacuum_gripper_turn_on(mVacuumGripper);
    else
      wb_vacuum_gripper_turn_off(mVacuumGripper);
  }

  void Ros2VacuumGripper::isOnCallback(const std::shared_ptr<webots_ros2_msgs::srv::GetBool::Request> request,
                                       std::shared_ptr<webots_ros2_msgs::srv::GetBool::Response> response) {
    response->value = wb_vacuum_gripper_is_on(mVacuumGripper);
  }

  void Ros2VacuumGripper::step() {
    if (!preStep())
      return;

    if (mIsPresenceEnabled)
      publishPresence();

    if (mAlwaysOn)
      return;

    // Enable/Disable sensor
    const bool shouldBeEnabled = mPresencePublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsPresenceEnabled) {
      if (shouldBeEnabled)
        wb_vacuum_gripper_enable_presence(mVacuumGripper, mPublishTimestepSyncedMs);
      else
        wb_vacuum_gripper_disable_presence(mVacuumGripper);
      mIsPresenceEnabled = shouldBeEnabled;
    }
  }

  void Ros2VacuumGripper::publishPresence() {
    mPresenceMessage.header.stamp = mNode->get_clock()->now();
    mPresenceMessage.data = wb_vacuum_gripper_get_presence(mVacuumGripper);
    mPresencePublisher->publish(mPresenceMessage);
  }
}  // namespace webots_ros2_driver
