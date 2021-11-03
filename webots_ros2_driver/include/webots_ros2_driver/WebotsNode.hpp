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
#include <unordered_map>

#include <tinyxml2.h>

#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <pluginlib/class_loader.hpp>

#include "webots_ros2_driver/PluginInterface.hpp"

namespace webots_ros2_driver
{
  class PluginInterface;

  class WebotsNode : public rclcpp::Node
  {
  public:
    WebotsNode(std::string name, webots::Supervisor *robot);
    void init();
    webots::Supervisor *robot() { return mRobot; }
    std::string urdf() const { return mRobotDescription; };

  private:
    void timerCallback();
    std::unordered_map<std::string, std::string> getDeviceRosProperties(const std::string &name) const;
    std::unordered_map<std::string, std::string> getPluginProperties(tinyxml2::XMLElement *pluginElement) const;
    void setAnotherNodeParameter(std::string anotherNodeName, std::string parameterName, std::string parameterValue);
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr mClient;

    std::string mRobotName;
    std::string mRobotDescription;

    rclcpp::TimerBase::SharedPtr mTimer;
    int mStep;
    webots::Supervisor *mRobot;
    std::vector<std::shared_ptr<PluginInterface>> mPlugins;
    pluginlib::ClassLoader<PluginInterface> mPluginLoader;
    tinyxml2::XMLElement *mWebotsXMLElement;
    std::shared_ptr<tinyxml2::XMLDocument> mRobotDescriptionDocument;
    std::shared_ptr<PluginInterface> loadPlugin(const std::string &type);

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr mClockPublisher;
    rosgraph_msgs::msg::Clock mClockMessage;
  };

} // end namespace webots_ros2_driver

#endif
