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

#include "webots_ros2_cpp/WebotsNode.hpp"

#include <webots/Device.hpp>

#include "webots_ros2_cpp/PluginInterface.hpp"
#include <webots_ros2_cpp/devices/Ros2Lidar.hpp>
#include <webots_ros2_cpp/devices/Ros2Camera.hpp>
#include <webots_ros2_cpp/devices/Ros2GPS.hpp>
#include <webots_ros2_cpp/devices/Ros2RangeFinder.hpp>
#include <webots_ros2_cpp/devices/Ros2DistanceSensor.hpp>
#include <webots_ros2_cpp/devices/Ros2LightSensor.hpp>

#include "webots_ros2_cpp/PluginInterface.hpp"

namespace webots_ros2
{
  const char *gDeviceRefferenceAttribute = "reference";
  const char *gDeviceRosTag = "ros";

  typedef std::shared_ptr<PluginInterface> (*creatorFunction)(webots_ros2::WebotsNode *node, const std::map<std::string, std::string> &parameters);

  WebotsNode::WebotsNode() : Node("webots_ros2")
  {
    mRobotDescription = this->declare_parameter<std::string>("robot_description", "");
    if (mRobotDescription != "")
    {
      mRobotDescriptionDocument = std::make_shared<tinyxml2::XMLDocument>();
      mRobotDescriptionDocument->Parse(mRobotDescription.c_str());
      if (!mRobotDescriptionDocument)
        throw std::runtime_error("Invalid URDF, it cannot be parsed");
      tinyxml2::XMLElement *robotXMLElement = mRobotDescriptionDocument->FirstChildElement("robot");
      if (!robotXMLElement)
        throw std::runtime_error("Invalid URDF, it doesn't contain a <robot> tag");
      mWebotsXMLElement = robotXMLElement->FirstChildElement("webots");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Robot description is not passed, using default parameters.");
    }
  }

  std::map<std::string, std::string> WebotsNode::getPluginProperties(tinyxml2::XMLElement *pluginElement)
  {
    std::map<std::string, std::string> properties;
    return properties;
  }

  std::map<std::string, std::string> WebotsNode::getDeviceRosProperties(const std::string &name)
  {
    std::map<std::string, std::string> properties({{"enabled", "true"}});

    // No URDF file specified
    if (!mWebotsXMLElement)
      return properties;

    tinyxml2::XMLElement *deviceChild = mWebotsXMLElement->FirstChildElement();
    while (deviceChild)
    {
      if (deviceChild->Attribute(gDeviceRefferenceAttribute) && deviceChild->Attribute(gDeviceRefferenceAttribute) == name)
        break;
      deviceChild = deviceChild->NextSiblingElement();
    }

    // No properties found for the given device
    if (!deviceChild || !deviceChild->FirstChildElement(gDeviceRosTag))
      return properties;

    // Store ROS properties
    tinyxml2::XMLElement *propertyChild = deviceChild->FirstChildElement(gDeviceRosTag)->FirstChildElement();
    while (propertyChild)
    {
      properties[propertyChild->Name()] = propertyChild->GetText();
      propertyChild = propertyChild->NextSiblingElement();
    }

    return properties;
  }

  void WebotsNode::init()
  {
    mRobot = new webots::Supervisor();
    mStep = mRobot->getBasicTimeStep();
    mTimer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&WebotsNode::timerCallback, this));

    // Load static plugins
    for (int i = 0; i < mRobot->getNumberOfDevices(); i++)
    {
      webots::Device *device = mRobot->getDeviceByIndex(i);

      // Prepare parameters
      std::map<std::string, std::string> parameters = getDeviceRosProperties(device->getName());
      if (parameters["enabled"] == "false")
        continue;
      parameters["name"] = device->getName();

      std::shared_ptr<PluginInterface> plugin = nullptr;
      switch (device->getNodeType())
      {
      case webots::Node::LIDAR:
        plugin = std::make_shared<webots_ros2::Ros2Lidar>();
        break;
      case webots::Node::CAMERA:
        plugin = std::make_shared<webots_ros2::Ros2Camera>();
        break;
      case webots::Node::GPS:
        plugin = std::make_shared<webots_ros2::Ros2GPS>();
        break;
      case webots::Node::RANGE_FINDER:
        plugin = std::make_shared<webots_ros2::Ros2RangeFinder>();
        break;
      case webots::Node::DISTANCE_SENSOR:
        plugin = std::make_shared<webots_ros2::Ros2DistanceSensor>();
        break;
      case webots::Node::LIGHT_SENSOR:
        plugin = std::make_shared<webots_ros2::Ros2LightSensor>();
        break;
      }
      if (plugin)
      {
        plugin->init(this, parameters);
        mPlugins.push_back(plugin);
      }
    }

    // Load dynamic plugins
    tinyxml2::XMLElement *pluginElement = mWebotsXMLElement->FirstChildElement("plugin");
    while (pluginElement)
    {
      if (!pluginElement->Attribute("type"))
        throw std::runtime_error("Invalid URDF, a plugin is missing a `type` property at line " + std::to_string(pluginElement->GetLineNum()));
      if (!pluginElement->Attribute("package"))
        throw std::runtime_error("Invalid URDF, a plugin is missing a `package` property at line " + std::to_string(pluginElement->GetLineNum()));

      const std::string type = pluginElement->Attribute("type");
      const std::string package = pluginElement->Attribute("package");

      pluginlib::ClassLoader<PluginInterface> *pluginLoader = new pluginlib::ClassLoader<PluginInterface>(package, "webots_ros2::PluginInterface");
      mPluginLoaders.push_back(std::shared_ptr<pluginlib::ClassLoader<PluginInterface>>(pluginLoader));

      std::shared_ptr<PluginInterface> plugin(pluginLoader->createUnmanagedInstance(type));
      std::map<std::string, std::string> pluginProperties = getPluginProperties(pluginElement);
      plugin->init(this, pluginProperties);
      mPlugins.push_back(plugin);

      pluginElement = pluginElement->NextSiblingElement("plugin");
    }
  }

  void WebotsNode::timerCallback()
  {
    mRobot->step(mStep);
    for (std::shared_ptr<PluginInterface> plugin : mPlugins)
      plugin->step();
  }
} // end namespace webots_ros2
