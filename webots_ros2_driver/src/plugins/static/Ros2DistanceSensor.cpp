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

    mLookupTable.assign(mDistanceSensor->getLookupTable(), mDistanceSensor->getLookupTable() + mDistanceSensor->getLookupTableSize() * 3);

    const int size = mLookupTable.size();
    const double maxValue = std::max(mLookupTable[0], mLookupTable[size - 3]);
    const double minValue = std::min(mLookupTable[0], mLookupTable[size - 3]);
    const double lowerStd = (mLookupTable[0] < mLookupTable[size - 3]) ? mLookupTable[2] * mLookupTable[0] : mLookupTable[size - 1] * mLookupTable[size - 3];
    const double upperStd = (mLookupTable[0] > mLookupTable[size - 3]) ? mLookupTable[2] * mLookupTable[0] : mLookupTable[size - 1] * mLookupTable[size - 3];
    const double minRange = minValue + lowerStd;
    const double maxRange = maxValue - upperStd;

    mPublisher = mNode->create_publisher<sensor_msgs::msg::Range>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mMessage.header.frame_id = mFrameName;
    mMessage.field_of_view = mDistanceSensor->getAperture();
    mMessage.min_range = minRange;
    mMessage.max_range = maxRange;
    mMessage.radiation_type = sensor_msgs::msg::Range::INFRARED;

    if (mAlwaysOn) {
      mDistanceSensor->enable(mPublishTimestepSyncedMs);
      mIsEnabled = true;
    }
  }

  void Ros2DistanceSensor::step()
  {
    if (!preStep())
      return;

    if (mIsEnabled)
      publishRange();

    if (mAlwaysOn)
      return;

    // Enable/Disable sensor
    const bool shouldBeEnabled = mPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mDistanceSensor->enable(mPublishTimestepSyncedMs);
      else
        mDistanceSensor->disable();
      mIsEnabled = shouldBeEnabled;
    }
  }

  void Ros2DistanceSensor::publishRange()
  {
    const double value = mDistanceSensor->getValue();
    if (std::isnan(value))
      return;
    mMessage.header.stamp = mNode->get_clock()->now();
    mMessage.range = interpolateLookupTable(value, mLookupTable);
    mPublisher->publish(mMessage);
  }
}
