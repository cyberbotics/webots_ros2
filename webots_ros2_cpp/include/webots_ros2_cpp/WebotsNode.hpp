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

#ifndef WEBOTS_NODE_HPP
#define WEBOTS_NODE_HPP

#include <webots_ros2_cpp/PluginInterface.hpp>

// webots
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// std
#include <memory>
#include <chrono>

namespace webots_ros2
{

  class WebotsNode : public rclcpp::Node
  {
  public:
    WebotsNode();
    void registerPlugin(const std::string &pathToPlugin, const std::map<std::string, std::string> &parameters);
    void init();
    std::shared_ptr<webots::Supervisor> robot() { return mRobot; }

  private:
    void timerCallback();
    static std::string fixedNameString(const std::string &name);

    std::string mRobotName;
    rclcpp::TimerBase::SharedPtr mTimer;
    int mStep;
    std::shared_ptr<webots::Supervisor> mRobot;
    std::vector<std::shared_ptr<webots_ros2::PluginInterface>> mPlugins;
  };

} // end namespace webots_ros2

#endif