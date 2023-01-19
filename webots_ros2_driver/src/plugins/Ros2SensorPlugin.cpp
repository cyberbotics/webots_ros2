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

#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>
#include <webots_ros2_driver/utils/Utils.hpp>

#include <webots/robot.h>

namespace webots_ros2_driver {
  void Ros2SensorPlugin::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    mNode = node;
    mLastUpdate = 0;

    mTopicName = parameters.count("topicName") ? parameters["topicName"] : "~/" + getFixedNameString(parameters["name"]);
    mPublishTimestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;
    mAlwaysOn = parameters.count("alwaysOn") ? (parameters["alwaysOn"] == "true") : false;
    mFrameName = parameters.count("frameName") ? parameters["frameName"] : getFixedNameString(parameters["name"]);

    // Calculate timestep
    mPublishTimestepSyncedMs = getDeviceTimestepMsFromPublishTimestep(mPublishTimestep, wb_robot_get_basic_time_step());
  }

  bool Ros2SensorPlugin::preStep() {
    // Update only if needed
    if (wb_robot_get_time() - mLastUpdate < mPublishTimestep)
      return false;
    mLastUpdate = wb_robot_get_time();
    return true;
  }
}  // namespace webots_ros2_driver
