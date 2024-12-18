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

#include "webots_ros2_driver/WebotsNode.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/timer.hpp>

#include <webots/device.h>
#include <webots/robot.h>

#include <webots_ros2_driver/plugins/static/Ros2Camera.hpp>
#include <webots_ros2_driver/plugins/static/Ros2Compass.hpp>
#include <webots_ros2_driver/plugins/static/Ros2DistanceSensor.hpp>
#include <webots_ros2_driver/plugins/static/Ros2Emitter.hpp>
#include <webots_ros2_driver/plugins/static/Ros2GPS.hpp>
#include <webots_ros2_driver/plugins/static/Ros2LED.hpp>
#include <webots_ros2_driver/plugins/static/Ros2Lidar.hpp>
#include <webots_ros2_driver/plugins/static/Ros2LightSensor.hpp>
#include <webots_ros2_driver/plugins/static/Ros2Pen.hpp>
#include <webots_ros2_driver/plugins/static/Ros2RangeFinder.hpp>
#include <webots_ros2_driver/plugins/static/Ros2Receiver.hpp>
#include <webots_ros2_driver/plugins/static/Ros2VacuumGripper.hpp>
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/PythonPlugin.hpp"

namespace webots_ros2_driver {
  const char *gDeviceReferenceAttribute = "reference";
  const char *gDeviceRosTag = "ros";
  const char *gPluginInterface = "webots_ros2_driver::PluginInterface";
  const char *gPluginInterfaceName = "webots_ros2_driver";

  bool gShutdownSignalReceived = false;

  void handleSigint(int sig) {
    gShutdownSignalReceived = true;
  }

  WebotsNode::WebotsNode(std::string name, rclcpp::NodeOptions &options) :
    Node(name, options),
    mPluginLoader(gPluginInterfaceName, gPluginInterface) {
    mRobotDescriptionParam = this->declare_parameter<std::string>("robot_description", "");
    mSetRobotStatePublisher = this->declare_parameter<bool>("set_robot_state_publisher", false);
    if (mRobotDescriptionParam != "") {
      mRobotDescriptionDocument = std::make_shared<tinyxml2::XMLDocument>();
      // Path to URDF file
      if (mRobotDescriptionParam.rfind(".urdf") == mRobotDescriptionParam.size() - 5)
        mRobotDescriptionDocument->LoadFile(mRobotDescriptionParam.c_str());
      // Path to Xacro file
      else if (mRobotDescriptionParam.rfind(".xacro") == mRobotDescriptionParam.size() - 6) {
        std::vector<std::string> xacroMappings =
          this->declare_parameter<std::vector<std::string>>("xacro_mappings", std::vector<std::string>());
        std::string command = "ros2 run xacro xacro " + mRobotDescriptionParam;
        for (const std::string &xacroMapping : xacroMappings) {
          command += " ";
          command += xacroMapping;
        }

        FILE *stream = popen(command.c_str(), "r");
        if (!stream)
          throw std::runtime_error("Failed to execute xacro command");

        char buffer[4096];
        std::string xacroOutput;
        while (fgets(buffer, sizeof(buffer), stream) != NULL)
          xacroOutput += buffer;
        pclose(stream);

        mRobotDescriptionDocument->Parse(xacroOutput.c_str());
      }
      // Full string (deprecated)
      else {
        mRobotDescriptionDocument->Parse(mRobotDescriptionParam.c_str());
        RCLCPP_WARN(
          get_logger(),
          "\033[33mPassing robot description as a string is deprecated. Provide the URDF or Xacro file path instead.\033[0m");
      }
      if (!mRobotDescriptionDocument)
        throw std::runtime_error("Invalid robot description, it cannot be parsed");
      // Access the robot and webots elements as needed
      tinyxml2::XMLElement *robotXMLElement = mRobotDescriptionDocument->FirstChildElement("robot");
      if (!robotXMLElement)
        throw std::runtime_error("Invalid URDF, it doesn't contain a <robot> tag");
      mWebotsXMLElement = robotXMLElement->FirstChildElement("webots");
    } else {
      mWebotsXMLElement = NULL;
      RCLCPP_INFO(get_logger(), "Robot description is not passed, using default parameters.");
    }
    // Store robot description string in mRobotDescription
    tinyxml2::XMLPrinter printer;
    mRobotDescriptionDocument->Accept(&printer);
    mRobotDescription = printer.CStr();

    mRemoveUrdfRobotPublisher = create_publisher<std_msgs::msg::String>("/remove_urdf_robot", rclcpp::ServicesQoS());
    mRemoveUrdfRobotMessage.data = name;

    // Path to component remapping YAML
    mComponentsRemappingFilePath = this->declare_parameter<std::string>("components_remappings", "");
    if (mComponentsRemappingFilePath != "") {
      std::ifstream file(mComponentsRemappingFilePath);
      if (!file)
        throw std::runtime_error("YAML file for components remappings doesn't exist.");

      YAML::Node root = YAML::Load(file);
      for (const auto &node : root) {
        const std::string key = node.first.as<std::string>();
        const std::string value = node.second.as<std::string>();
        mComponentsRemapping[key] = value;
      }
    }
  }

  std::unordered_map<std::string, std::string> WebotsNode::getPluginProperties(tinyxml2::XMLElement *pluginElement) const {
    std::unordered_map<std::string, std::string> properties;

    if (pluginElement) {
      tinyxml2::XMLElement *deviceChild = pluginElement->FirstChildElement();
      while (deviceChild) {
        properties[deviceChild->Name()] = deviceChild->GetText();
        deviceChild = deviceChild->NextSiblingElement();
      }
    }

    return properties;
  }

  std::unordered_map<std::string, std::string> WebotsNode::getDeviceRosProperties(const std::string &name) const {
    std::unordered_map<std::string, std::string> properties({{"enabled", "true"}});

    // No URDF file specified
    if (!mWebotsXMLElement)
      return properties;

    tinyxml2::XMLElement *deviceChild = mWebotsXMLElement->FirstChildElement();
    while (deviceChild) {
      if (deviceChild->Attribute(gDeviceReferenceAttribute) && deviceChild->Attribute(gDeviceReferenceAttribute) == name)
        break;
      deviceChild = deviceChild->NextSiblingElement();
    }

    // No properties found for the given device
    if (!deviceChild || !deviceChild->FirstChildElement(gDeviceRosTag))
      return properties;

    // Store ROS properties
    tinyxml2::XMLElement *propertyChild = deviceChild->FirstChildElement(gDeviceRosTag)->FirstChildElement();
    while (propertyChild) {
      properties[propertyChild->Name()] = propertyChild->GetText();
      propertyChild = propertyChild->NextSiblingElement();
    }

    return properties;
  }

  void WebotsNode::replaceUrdfNames(std::string &urdf) {
    for (const auto &map : mComponentsRemapping) {
      const std::string searchStr1 = "name=\"" + map.first + "\"";
      const std::string searchStr2 = "link=\"" + map.first + "\"";
      size_t pos = std::min(urdf.find(searchStr1), urdf.find(searchStr2));
      while (pos != std::string::npos) {
        const size_t quoteStartPos = pos + 6;
        const size_t quoteEndPos = quoteStartPos + map.first.length();
        urdf.replace(quoteStartPos, quoteEndPos - quoteStartPos, map.second);
        pos = std::min(urdf.find(searchStr1, quoteEndPos), urdf.find(searchStr2, quoteEndPos));
      }
    }
  }

  void WebotsNode::init() {
    if (mSetRobotStatePublisher) {
      std::string prefix = "";
      std::string webotsUrdf = wb_robot_get_urdf(prefix.c_str());
      replaceUrdfNames(webotsUrdf);

      // Add <ros2_control /> and other tags to the Webots generated URDF
      const std::vector<std::string> tags = {"ros2_control", "webots"};
      std::shared_ptr<tinyxml2::XMLDocument> webotsUrdfModelDocument = std::make_shared<tinyxml2::XMLDocument>();
      webotsUrdfModelDocument->Parse(webotsUrdf.c_str());
      tinyxml2::XMLElement *webotsRobotXMLElement = webotsUrdfModelDocument->FirstChildElement("robot");
      tinyxml2::XMLElement *robotXMLElement = mRobotDescriptionDocument->FirstChildElement("robot");
      if (!robotXMLElement)
        throw std::runtime_error("Invalid URDF, it doesn't contain a <robot> tag");

      for (tinyxml2::XMLElement *child = robotXMLElement->FirstChildElement(); child; child = child->NextSiblingElement())
        if (std::find(tags.begin(), tags.end(), child->Name()) != tags.end())
          webotsRobotXMLElement->InsertEndChild(child->DeepClone(webotsUrdfModelDocument.get()));

      std::string expandedUrdf = "";
      tinyxml2::XMLPrinter printer;
      webotsUrdfModelDocument->Accept(&printer);
      expandedUrdf = printer.CStr();

      setAnotherNodeParameter("robot_state_publisher", "robot_description", expandedUrdf);
    }

    mStep = wb_robot_get_basic_time_step();

    // Load static plugins
    // Static plugins are automatically configured based on the robot model.
    // The static plugins will try to guess parameter based on the robot model,
    // but one can overwrite the default behavior in the <webots> section.
    // Typical static plugins are ROS 2 interfaces for Webots devices.
    for (int i = 0; i < wb_robot_get_number_of_devices(); i++) {
      WbDeviceTag device = wb_robot_get_device_by_index(i);

      // Prepare parameters
      std::unordered_map<std::string, std::string> parameters = getDeviceRosProperties(wb_device_get_name(device));
      if (parameters["enabled"] == "false")
        continue;
      parameters["name"] = wb_device_get_name(device);

      std::shared_ptr<PluginInterface> plugin = nullptr;
      switch (wb_device_get_node_type(device)) {
        case WB_NODE_LIDAR:
          plugin = std::make_shared<webots_ros2_driver::Ros2Lidar>();
          break;
        case WB_NODE_CAMERA:
          plugin = std::make_shared<webots_ros2_driver::Ros2Camera>();
          break;
        case WB_NODE_GPS:
          plugin = std::make_shared<webots_ros2_driver::Ros2GPS>();
          break;
        case WB_NODE_RANGE_FINDER:
          plugin = std::make_shared<webots_ros2_driver::Ros2RangeFinder>();
          break;
        case WB_NODE_DISTANCE_SENSOR:
          plugin = std::make_shared<webots_ros2_driver::Ros2DistanceSensor>();
          break;
        case WB_NODE_LIGHT_SENSOR:
          plugin = std::make_shared<webots_ros2_driver::Ros2LightSensor>();
          break;
        case WB_NODE_LED:
          plugin = std::make_shared<webots_ros2_driver::Ros2LED>();
          break;
        case WB_NODE_PEN:
          plugin = std::make_shared<webots_ros2_driver::Ros2Pen>();
          break;
        case WB_NODE_EMITTER:
          plugin = std::make_shared<webots_ros2_driver::Ros2Emitter>();
          break;
        case WB_NODE_RECEIVER:
          plugin = std::make_shared<webots_ros2_driver::Ros2Receiver>();
          break;
        case WB_NODE_COMPASS:
          plugin = std::make_shared<webots_ros2_driver::Ros2Compass>();
          break;
        case WB_NODE_VACUUM_GRIPPER:
          plugin = std::make_shared<webots_ros2_driver::Ros2VacuumGripper>();
          break;
      }
      if (plugin) {
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
    while (pluginElement) {
      if (!pluginElement->Attribute("type"))
        throw std::runtime_error("Invalid URDF, a plugin is missing a `type` property at line " +
                                 std::to_string(pluginElement->GetLineNum()));

      const std::string type = pluginElement->Attribute("type");

      std::shared_ptr<PluginInterface> plugin = loadPlugin(type);
      std::unordered_map<std::string, std::string> pluginProperties = getPluginProperties(pluginElement);
      plugin->init(this, pluginProperties);
      mPlugins.push_back(plugin);

      pluginElement = pluginElement->NextSiblingElement("plugin");
    }
  }

  std::shared_ptr<PluginInterface> WebotsNode::loadPlugin(const std::string &type) {
    // First, we assume the plugin is C++
    try {
      std::shared_ptr<PluginInterface> plugin(mPluginLoader.createUnmanagedInstance(type));
      return plugin;
    } catch (const pluginlib::LibraryLoadException &e) {
      // It may be a Python plugin
    } catch (const pluginlib::CreateClassException &e) {
      throw std::runtime_error("The " + type + " class cannot be initialized.");
    }

    std::shared_ptr<PluginInterface> plugin = PythonPlugin::createFromType(type);
    if (plugin == NULL)
      throw std::runtime_error("The " + type + " plugin cannot be found (C++ or Python).");

    return plugin;
  }

  int WebotsNode::step() {
    if (gShutdownSignalReceived && !mWaitingForUrdfRobotToBeRemoved) {
      mRemoveUrdfRobotPublisher->publish(mRemoveUrdfRobotMessage);
      mWaitingForUrdfRobotToBeRemoved = true;
      return -1;
    }

    const int result = wb_robot_step(mStep);
    if (result == -1)
      return result;
    for (std::shared_ptr<PluginInterface> plugin : mPlugins)
      plugin->step();

    return result;
  }

  void WebotsNode::setAnotherNodeParameter(std::string anotherNodeName, std::string parameterName, std::string parameterValue) {
    mClient = create_client<rcl_interfaces::srv::SetParameters>(anotherNodeName + "/set_parameters");
    mClient->wait_for_service(std::chrono::seconds(1));
    rcl_interfaces::srv::SetParameters::Request::SharedPtr request =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = parameterName;
    parameter.value.string_value = parameterValue;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    request->parameters.push_back(parameter);
    mClient->async_send_request(request);
  }

  void WebotsNode::handleSignals() {
    signal(SIGINT, handleSigint);
  }
}  // end namespace webots_ros2_driver
