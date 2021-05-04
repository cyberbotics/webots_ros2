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

#include <memory>
#include <chrono>
#include <map>

#include <tinyxml2.h>

#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <pluginlib/class_loader.hpp>

#include "webots_ros2_cpp/PluginInterface.hpp"

namespace webots_ros2
{

  class WebotsNode : public rclcpp::Node
  {
  public:
    WebotsNode();
    void init();
    webots::Supervisor *robot() { return mRobot; }

  private:
    void timerCallback();
    std::map<std::string, std::string> getDeviceRosProperties(const std::string &name);
    std::map<std::string, std::string> getPluginProperties(tinyxml2::XMLElement *pluginElement);

    std::string mRobotName;
    rclcpp::TimerBase::SharedPtr mTimer;
    int mStep;
    webots::Supervisor *mRobot;
    std::vector<std::shared_ptr<PluginInterface>> mPlugins;
    std::vector<std::shared_ptr<pluginlib::ClassLoader<PluginInterface>>> mPluginLoaders;
    tinyxml2::XMLElement *mWebotsXMLElement;
    std::shared_ptr<tinyxml2::XMLDocument> mRobotDescriptionDocument;
  };

} // end namespace webots_ros2

#endif