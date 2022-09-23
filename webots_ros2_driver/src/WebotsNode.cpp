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

#include "webots_ros2_driver/WebotsNode.hpp"

#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/timer.hpp>

#include <webots/Device.hpp>

#include "webots_ros2_driver/PluginInterface.hpp"
#include <webots_ros2_driver/plugins/static/Ros2Lidar.hpp>
#include <webots_ros2_driver/plugins/static/Ros2Camera.hpp>
#include <webots_ros2_driver/plugins/static/Ros2GPS.hpp>
#include <webots_ros2_driver/plugins/static/Ros2RangeFinder.hpp>
#include <webots_ros2_driver/plugins/static/Ros2DistanceSensor.hpp>
#include <webots_ros2_driver/plugins/static/Ros2LightSensor.hpp>
#include <webots_ros2_driver/plugins/static/Ros2LED.hpp>

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/PythonPlugin.hpp"

namespace webots_ros2_driver
{
  const char *gDeviceReferenceAttribute = "reference";
  const char *gDeviceRosTag = "ros";
  const char *gPluginInterface = "webots_ros2_driver::PluginInterface";
  const char *gPluginInterfaceName = "webots_ros2_driver";

  bool gShutdownSignalReceived = false;

  void handleSigint(int sig)
  {
    gShutdownSignalReceived = true;
  }

  WebotsNode::WebotsNode(std::string name, webots::Supervisor *robot) : Node(name), mRobot(robot), mPluginLoader(gPluginInterfaceName, gPluginInterface)
  {
    mRobotDescription = this->declare_parameter<std::string>("robot_description", "");
    mSetRobotStatePublisher = this->declare_parameter<bool>("set_robot_state_publisher", false);
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
      mWebotsXMLElement = NULL;
      RCLCPP_INFO(get_logger(), "Robot description is not passed, using default parameters.");
    }

    mRemoveUrdfRobotPublisher = create_publisher<std_msgs::msg::String>("/remove_urdf_robot", rclcpp::ServicesQoS());
    mRemoveUrdfRobotMessage.data = name;
  }

  std::unordered_map<std::string, std::string> WebotsNode::getPluginProperties(tinyxml2::XMLElement *pluginElement) const
  {
    std::unordered_map<std::string, std::string> properties;

    if (pluginElement)
    {
      tinyxml2::XMLElement *deviceChild = pluginElement->FirstChildElement();
      while (deviceChild)
      {
        properties[deviceChild->Name()] = deviceChild->GetText();
        deviceChild = deviceChild->NextSiblingElement();
      }
    }

    return properties;
  }

  std::unordered_map<std::string, std::string> WebotsNode::getDeviceRosProperties(const std::string &name) const
  {
    std::unordered_map<std::string, std::string> properties({{"enabled", "true"}});

    // No URDF file specified
    if (!mWebotsXMLElement)
      return properties;

    tinyxml2::XMLElement *deviceChild = mWebotsXMLElement->FirstChildElement();
    while (deviceChild)
    {
      if (deviceChild->Attribute(gDeviceReferenceAttribute) && deviceChild->Attribute(gDeviceReferenceAttribute) == name)
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
    if (mSetRobotStatePublisher)
      setAnotherNodeParameter("robot_state_publisher", "robot_description", mRobot->getUrdf());

    mStep = mRobot->getBasicTimeStep();

    // Load static plugins
    // Static plugins are automatically configured based on the robot model.
    // The static plugins will try to guess parameter based on the robot model,
    // but one can overwrite the default behavior in the <webots> section.
    // Typical static plugins are ROS 2 interfaces for Webots devices.
    for (int i = 0; i < mRobot->getNumberOfDevices(); i++)
    {
      webots::Device *device = mRobot->getDeviceByIndex(i);

      // Prepare parameters
      std::unordered_map<std::string, std::string> parameters = getDeviceRosProperties(device->getName());
      if (parameters["enabled"] == "false")
        continue;
      parameters["name"] = device->getName();

      std::shared_ptr<PluginInterface> plugin = nullptr;
      switch (device->getNodeType())
      {
      case webots::Node::LIDAR:
        plugin = std::make_shared<webots_ros2_driver::Ros2Lidar>();
        break;
      case webots::Node::CAMERA:
        plugin = std::make_shared<webots_ros2_driver::Ros2Camera>();
        break;
      case webots::Node::GPS:
        plugin = std::make_shared<webots_ros2_driver::Ros2GPS>();
        break;
      case webots::Node::RANGE_FINDER:
        plugin = std::make_shared<webots_ros2_driver::Ros2RangeFinder>();
        break;
      case webots::Node::DISTANCE_SENSOR:
        plugin = std::make_shared<webots_ros2_driver::Ros2DistanceSensor>();
        break;
      case webots::Node::LIGHT_SENSOR:
        plugin = std::make_shared<webots_ros2_driver::Ros2LightSensor>();
        break;
      case webots::Node::LED:
        plugin = std::make_shared<webots_ros2_driver::Ros2LED>();
        break;
      }
      if (plugin)
      {
        plugin->init(this, parameters);
        mPlugins.push_back(plugin);
      }
    }

    // Load dynamic plugins
    // Dynamic plugins are loaded only if specified in the <webots> section.
    // Those are user contributed plugins or ROS 2 intefaces for which we cannot guess default configuration.
    // Typical examples are ros2_control and IMU (there is not Webots IMU devices, but it is composed of three).
    if (!mWebotsXMLElement)
      return;

    tinyxml2::XMLElement *pluginElement = mWebotsXMLElement->FirstChildElement("plugin");
    while (pluginElement)
    {
      if (!pluginElement->Attribute("type"))
        throw std::runtime_error("Invalid URDF, a plugin is missing a `type` property at line " + std::to_string(pluginElement->GetLineNum()));

      const std::string type = pluginElement->Attribute("type");

      std::shared_ptr<PluginInterface> plugin = loadPlugin(type);
      std::unordered_map<std::string, std::string> pluginProperties = getPluginProperties(pluginElement);
      plugin->init(this, pluginProperties);
      mPlugins.push_back(plugin);

      pluginElement = pluginElement->NextSiblingElement("plugin");
    }
  }

  std::shared_ptr<PluginInterface> WebotsNode::loadPlugin(const std::string &type)
  {
    // First, we assume the plugin is C++
    try
    {
      std::shared_ptr<PluginInterface> plugin(mPluginLoader.createUnmanagedInstance(type));
      return plugin;
    }
    catch (const pluginlib::LibraryLoadException &e)
    {
      // It may be a Python plugin
    }
    catch (const pluginlib::CreateClassException &e)
    {
      throw std::runtime_error("The " + type + " class cannot be initialized.");
    }

    std::shared_ptr<PluginInterface> plugin = PythonPlugin::createFromType(type);
    if (plugin == NULL)
      throw std::runtime_error("The " + type + " plugin cannot be found (C++ or Python).");

    return plugin;
  }

  int WebotsNode::step()
  {
    if (gShutdownSignalReceived && !mWaitingForUrdfRobotToBeRemoved)
    {
      mRemoveUrdfRobotPublisher->publish(mRemoveUrdfRobotMessage);
      mWaitingForUrdfRobotToBeRemoved = true;
    }

    const int result = mRobot->step(mStep);
    if (result == -1)
      return result;
    for (std::shared_ptr<PluginInterface> plugin : mPlugins)
      plugin->step();

    return result;
  }

  void WebotsNode::setAnotherNodeParameter(std::string anotherNodeName, std::string parameterName, std::string parameterValue)
  {
    mClient = create_client<rcl_interfaces::srv::SetParameters>(get_namespace() + anotherNodeName + "/set_parameters");
    mClient->wait_for_service(std::chrono::seconds(1));
    rcl_interfaces::srv::SetParameters::Request::SharedPtr request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = parameterName;
    parameter.value.string_value = parameterValue;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    request->parameters.push_back(parameter);
    mClient->async_send_request(request);
  }

  void WebotsNode::handleSignals()
  {
    signal(SIGINT, handleSigint);
  }
} // end namespace webots_ros2_driver
