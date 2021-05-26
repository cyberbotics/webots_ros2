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

#include <webots_ros2_driver/plugins/static/Ros2DistanceSensor.hpp>

#include <webots_ros2_driver/utils/Math.hpp>

namespace webots_ros2_driver
{
  void Ros2DistanceSensor::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    Ros2SensorPlugin::init(node, parameters);
    mIsEnabled = false;
    mDistanceSensor = mNode->robot()->getDistanceSensor(parameters["name"]);

    assert(mDistanceSensor != NULL);

    mPublisher = mNode->create_publisher<sensor_msgs::msg::Range>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mMessage.header.frame_id = mFrameName;
    mMessage.field_of_view = mDistanceSensor->getAperture();
    mMessage.min_range = mDistanceSensor->getMinValue();
    mMessage.max_range = mDistanceSensor->getMaxValue();
    mMessage.radiation_type = sensor_msgs::msg::Range::INFRARED;

    mLookupTable.assign(mDistanceSensor->getLookupTable(), mDistanceSensor->getLookupTable() + mDistanceSensor->getLookupTableSize());
  }

  void Ros2DistanceSensor::step()
  {
    if (!preStep())
      return;

    // Enable/Disable sensor
    const bool imageSubscriptionsExist = mPublisher->get_subscription_count() > 0;
    const bool shouldBeEnabled = mAlwaysOn || imageSubscriptionsExist;

    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mDistanceSensor->enable(mPublishTimestepSyncedMs);
      else
        mDistanceSensor->disable();
      mIsEnabled = shouldBeEnabled;
    }

    // Publish data
    if (mAlwaysOn || imageSubscriptionsExist)
      publishRange();
  }

  void Ros2DistanceSensor::publishRange()
  {
    mMessage.header.stamp = rclcpp::Clock().now();
    mMessage.range = interpolateLookupTable(mDistanceSensor->getValue(), mLookupTable);
    mPublisher->publish(mMessage);
  }
}
