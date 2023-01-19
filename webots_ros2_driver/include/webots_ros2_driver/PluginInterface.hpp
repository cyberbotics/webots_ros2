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

#ifndef PLUGIN_INTERFACE
#define PLUGIN_INTERFACE

#include <string>
#include <unordered_map>

namespace webots_ros2_driver {
  class WebotsNode;

  class PluginInterface {
  public:
    /// Prepare your plugin in this method.
    /// Fired before the node is spinned.
    /// Parameters are passed from the WebotsNode and/or from URDF.
    /**
     * \param[in] node WebotsNode inherited from `rclcpp::Node` with a few extra methods related
     * \param[in] parameters Parameters (key-value pairs) located under a <plugin> (dynamically loaded plugins) or <ros>
     * (statically loaded plugins).
     */
    virtual void init(WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) = 0;

    /// This method is called on each timestep.
    /// You should not call `robot.step()` in this method as it is automatically called.
    virtual void step() = 0;
  };
}  // namespace webots_ros2_driver

#endif
