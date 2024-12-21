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

#ifndef WEBOTS_NODE_HPP
#define WEBOTS_NODE_HPP

#include <chrono>
#include <memory>
#include <unordered_map>

#include <tinyxml2.h>

#include <webots/supervisor.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "webots_ros2_driver/PluginInterface.hpp"

namespace webots_ros2_driver {
  class PluginInterface;

  class WebotsNode : public rclcpp::Node {
  public:
    WebotsNode(std::string name, rclcpp::NodeOptions &options);
    void init();
    int step();
    std::string urdf() const { return mRobotDescription; };
    static void handleSignals();
    std::vector<std::shared_ptr<PluginInterface>> mPlugins;

  private:
    std::unordered_map<std::string, std::string> getDeviceRosProperties(const std::string &name) const;
    std::unordered_map<std::string, std::string> getPluginProperties(tinyxml2::XMLElement *pluginElement) const;
    void setAnotherNodeParameter(std::string anotherNodeName, std::string parameterName, std::string parameterValue);
    void replaceUrdfNames(std::string &urdf);
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr mClient;

    std::string mRobotDescriptionParam;
    std::string mRobotDescription;
    bool mSetRobotStatePublisher;
    std::string mComponentsRemappingFilePath;
    std::unordered_map<std::string, std::string> mComponentsRemapping;

    bool mWaitingForUrdfRobotToBeRemoved = false;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mRemoveUrdfRobotPublisher;
    std_msgs::msg::String mRemoveUrdfRobotMessage;

    rclcpp::TimerBase::SharedPtr mTimer;
    int mStep;
    pluginlib::ClassLoader<PluginInterface> mPluginLoader;
    tinyxml2::XMLElement *mWebotsXMLElement;
    std::shared_ptr<tinyxml2::XMLDocument> mRobotDescriptionDocument;
    std::shared_ptr<PluginInterface> loadPlugin(const std::string &type);
  };

}  // end namespace webots_ros2_driver

#endif
