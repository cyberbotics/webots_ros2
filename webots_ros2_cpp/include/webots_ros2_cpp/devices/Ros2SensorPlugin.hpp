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

#ifndef ROS2_SENSOR_PLUGIN_HPP
#define ROS2_SENSOR_PLUGIN_HPP

#include <map>
#include <webots/RangeFinder.hpp>
#include <webots_ros2_cpp/PluginInterface.hpp>
#include <webots_ros2_cpp/WebotsNode.hpp>


namespace webots_ros2
{

  /// Utility class for common Webots sensors
  class Ros2SensorPlugin : public PluginInterface
  {
  public:
    void init(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters) override;

    /// Checks if a sensor has data ready and updates the last update time
    /**
    * \return Should publishing from the sensor be considered
    */
    bool preStep();

  protected:
    webots_ros2::WebotsNode *mNode;

    std::string mTopicName;
    std::string mFrameName;
    double mPublishTimestep;
    bool mAlwaysOn;
    int mPublishTimestepSyncedMs;

    double mLastUpdate;
  };
}

#endif
