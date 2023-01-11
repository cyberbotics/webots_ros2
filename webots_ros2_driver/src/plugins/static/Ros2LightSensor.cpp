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

#include <webots_ros2_driver/plugins/static/Ros2LightSensor.hpp>

#include <webots_ros2_driver/utils/Math.hpp>

#include <webots/robot.h>

namespace webots_ros2_driver {
  const double gIrradianceToIlluminance = 120.0;

  void Ros2LightSensor::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);
    mIsEnabled = false;
    mLightSensor = wb_robot_get_device(parameters["name"].c_str());

    assert(mLightSensor != 0);

    mPublisher = mNode->create_publisher<sensor_msgs::msg::Illuminance>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mMessage.header.frame_id = mFrameName;

    mLookupTable.assign(
      wb_light_sensor_get_lookup_table(mLightSensor),
      wb_light_sensor_get_lookup_table(mLightSensor) + wb_light_sensor_get_lookup_table_size(mLightSensor) * 3);

    if (mAlwaysOn) {
      wb_light_sensor_enable(mLightSensor, mPublishTimestepSyncedMs);
      mIsEnabled = true;
    }
  }

  void Ros2LightSensor::step() {
    if (!preStep())
      return;

    if (mIsEnabled)
      publishValue();

    if (mAlwaysOn)
      return;

    // Enable/Disable sensor
    const bool shouldBeEnabled = mPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled) {
      if (shouldBeEnabled)
        wb_light_sensor_enable(mLightSensor, mPublishTimestepSyncedMs);
      else
        wb_light_sensor_disable(mLightSensor);
      mIsEnabled = shouldBeEnabled;
    }
  }

  void Ros2LightSensor::publishValue() {
    const double value = wb_light_sensor_get_value(mLightSensor);
    mMessage.header.stamp = mNode->get_clock()->now();
    mMessage.illuminance = interpolateLookupTable(value, mLookupTable);
    mMessage.variance = findVariance(value);
    mPublisher->publish(mMessage);
  }

  double Ros2LightSensor::findVariance(double rawValue) {
    // Find relative standard deviation in lookup table
    double relativeStd = NAN;
    for (int i = 0; i < mLookupTable.size() - 3; i += 3)
      if ((mLookupTable[i + 1] < rawValue < mLookupTable[i + 3 + 1]) ||
          (mLookupTable[i + 1] > rawValue > mLookupTable[i + 3 + 1])) {
        relativeStd = mLookupTable[i + 2];
        break;
      }
    if (relativeStd == NAN)
      if (rawValue < mLookupTable[1])
        relativeStd = mLookupTable[2];
      else
        relativeStd = mLookupTable[-1];

    // Calculate variance from the relative standard deviation
    double std = interpolateLookupTable(rawValue, mLookupTable) * gIrradianceToIlluminance * relativeStd;
    return std * std;
  }
}  // namespace webots_ros2_driver
