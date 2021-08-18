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

#include <webots_ros2_driver/plugins/dynamic/Ros2IMU.hpp>

#include <webots_ros2_driver/utils/Math.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace webots_ros2_driver
{
  void Ros2IMU::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    Ros2SensorPlugin::init(node, parameters);
    mIsEnabled = false;
    mInertialUnit = NULL;
    mGyro = NULL;
    mAccelerometer = NULL;

    if (!parameters.count("inertialUnitName") && !parameters.count("gyroName") && !parameters.count("accelerometerName"))
      throw std::runtime_error("The IMU plugins has to contain at least of: <inertialUnitName>, <gyroName>, or <accelerometerName>");

    if (parameters.count("inertialUnitName"))
    {
      mInertialUnit = mNode->robot()->getInertialUnit(parameters["inertialUnitName"]);
      if (mInertialUnit == NULL)
        throw std::runtime_error("Cannot find InertialUnit with name " + parameters["inertialUnitName"]);
    }

    if (parameters.count("gyroName"))
    {
      mGyro = mNode->robot()->getGyro(parameters["gyroName"]);
      if (mGyro == NULL)
        throw std::runtime_error("Cannot find Gyro with name " + parameters["gyroName"]);
    }

    if (parameters.count("accelerometerName"))
    {
      mAccelerometer = mNode->robot()->getAccelerometer(parameters["accelerometerName"]);
      if (mAccelerometer == NULL)
        throw std::runtime_error("Cannot find Accelerometer with name " + parameters["accelerometerName"]);
    }

    mPublisher = mNode->create_publisher<sensor_msgs::msg::Imu>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mMessage.header.frame_id = mFrameName;

    if (mAlwaysOn) {
      enable();
      mIsEnabled = true;
    }
  }

  void Ros2IMU::enable()
  {
    if (mInertialUnit)
      mInertialUnit->enable(mPublishTimestepSyncedMs);
    if (mAccelerometer)
      mAccelerometer->enable(mPublishTimestepSyncedMs);
    if (mGyro)
      mGyro->enable(mPublishTimestepSyncedMs);
  }

  void Ros2IMU::disable()
  {
    if (mInertialUnit)
      mInertialUnit->disable();
    if (mAccelerometer)
      mAccelerometer->disable();
    if (mGyro)
      mGyro->disable();
  }

  void Ros2IMU::step()
  {
    if (!preStep())
      return;

    if (mIsEnabled)
      publishData();

    if (mAlwaysOn)
      return;

    // Enable/Disable sensor
    const bool shouldBeEnabled = mPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        enable();
      else
        disable();
      mIsEnabled = shouldBeEnabled;
    }
  }

  void Ros2IMU::publishData()
  {
    mMessage.header.stamp = mNode->get_clock()->now();
    if (mAccelerometer)
    {
      const double *values = mAccelerometer->getValues();
      mMessage.linear_acceleration.x = values[0];
      mMessage.linear_acceleration.y = values[1];
      mMessage.linear_acceleration.z = values[2];
    }
    if (mGyro)
    {
      const double *values = mGyro->getValues();
      mMessage.angular_velocity.x = values[0];
      mMessage.angular_velocity.y = values[1];
      mMessage.angular_velocity.z = values[2];
    }
    if (mInertialUnit)
    {
      const double *values = mInertialUnit->getQuaternion();
      mMessage.orientation.x = values[0];
      mMessage.orientation.y = values[1];
      mMessage.orientation.z = values[2];
      mMessage.orientation.w = values[3];
    }
    mPublisher->publish(mMessage);
  }
}

PLUGINLIB_EXPORT_CLASS(webots_ros2_driver::Ros2IMU, webots_ros2_driver::PluginInterface)
