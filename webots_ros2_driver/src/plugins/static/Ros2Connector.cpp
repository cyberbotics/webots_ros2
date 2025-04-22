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

#include "webots_ros2_driver/plugins/static/Ros2Connector.hpp"

#include <webots/connector.h>
#include <webots/robot.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace webots_ros2_driver {
  void Ros2Connector::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);

    // This parameter is read when loading the URDF file
    mDeviceName = parameters.count("name") ? parameters["name"] : "connector";
    mConnector = wb_robot_get_device(mDeviceName.c_str());

    assert(mConnector != 0);

    // Initialize services, publishers and subcriptions
    mLockSubscription = mNode->create_subscription<std_msgs::msg::Bool>(
      mTopicName + "/lock", rclcpp::SensorDataQoS().reliable(), std::bind(&Ros2Connector::LockCallback, this, _1));

    mLockService = mNode->create_service<webots_ros2_msgs::srv::GetBool>(
      mTopicName + "/is_locked", std::bind(&Ros2Connector::isLockedCallback, this, _1, _2));

    mIsPresenceEnabled = false;
    mPresencePublisher =
      mNode->create_publisher<webots_ros2_msgs::msg::IntStamped>(mTopicName + "/presence", rclcpp::SensorDataQoS().reliable());
  }

  void Ros2Connector::LockCallback(const std_msgs::msg::Bool::SharedPtr message) {
    if (message->data)
      wb_connector_lock(mConnector);
    else
      wb_connector_unlock(mConnector);
  }

  void Ros2Connector::isLockedCallback(const std::shared_ptr<webots_ros2_msgs::srv::GetBool::Request> request,
                                       std::shared_ptr<webots_ros2_msgs::srv::GetBool::Response> response) {
    response->value = wb_connector_is_locked(mConnector);
  }

  void Ros2Connector::step() {
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
        wb_connector_enable_presence(mConnector, mPublishTimestepSyncedMs);
      else
        wb_connector_disable_presence(mConnector);
      mIsPresenceEnabled = shouldBeEnabled;
    }
  }

  void Ros2Connector::publishPresence() {
    mPresenceMessage.header.stamp = mNode->get_clock()->now();
    mPresenceMessage.data = wb_connector_get_presence(mConnector);
    mPresencePublisher->publish(mPresenceMessage);
  }
}  // namespace webots_ros2_driver
