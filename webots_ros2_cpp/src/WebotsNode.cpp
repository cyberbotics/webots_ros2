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

#include <dlfcn.h>
#include <webots/Device.hpp>

#include "webots_ros2_cpp/WebotsNode.hpp"
#include "webots_ros2_cpp/PluginInterface.hpp"
#include <webots_ros2_cpp/devices/Ros2Lidar.hpp>

namespace webots_ros2
{
  typedef std::shared_ptr<PluginInterface> (*creatorFunction)(webots_ros2::WebotsNode *node, const std::map<std::string, std::string> &parameters);

  WebotsNode::WebotsNode() : Node("webots_ros2_interface")
  {
    const std::string robotDescription = this->declare_parameter<std::string>("robot_description", "");
    mRobotDescriptionDocument = std::make_shared<tinyxml2::XMLDocument>();
    mRobotDescriptionDocument->Parse(robotDescription.c_str());
    mWebotsXMLElement = mRobotDescriptionDocument->FirstChildElement("webots");
  }

  std::map<std::string, std::string> WebotsNode::getDeviceRosProperties(const std::string &name)
  {
    std::map<std::string, std::string> properties({{"enabled", "true"}});

    // No URDF file specified
    if (!mWebotsXMLElement)
      return properties;

    tinyxml2::XMLElement *deviceChild = mWebotsXMLElement->FirstChildElement();
    while (deviceChild) {
      if (deviceChild->Attribute("reference") != NULL && deviceChild->Attribute("reference") == name)
        break;
      deviceChild = deviceChild->NextSiblingElement();
    }

    // No properties found for the given device
    if (deviceChild == NULL || deviceChild->FirstChildElement("ros") == NULL)
      return properties;

    // Store ROS properties
    tinyxml2::XMLElement *propertyChild = deviceChild->FirstChildElement("ros")->FirstChildElement();
    while (propertyChild) {
      properties[propertyChild->Name()] = propertyChild->GetText();
      propertyChild = propertyChild->NextSiblingElement();
    }

    return properties;
  }

  void WebotsNode::init()
  {
    mRobot = new webots::Supervisor();
    std::chrono::milliseconds ms((int)mRobot->getBasicTimeStep());
    mTimer = this->create_wall_timer(ms, std::bind(&WebotsNode::timerCallback, this));

    for (int i = 0; i < mRobot->getNumberOfDevices(); i++)
    {
      webots::Device *device = mRobot->getDeviceByIndex(i);

      // Prepare parameters
      std::map<std::string, std::string> parameters = getDeviceRosProperties(device->getName());
      if (parameters["enabled"] == "false")
        continue;
      parameters["name"] = device->getName();

      switch (device->getNodeType())
      {
      case webots::Node::LIDAR:
      {
        auto lidar = std::make_shared<webots_ros2::Ros2Lidar>(this, parameters);
        mPlugins.push_back(lidar);
        break;
      }
        /*
      case webots::Node::CAMERA:
      {
        auto camera = std::make_shared<wb_ros2_interface::sensors::WbRos2Camera>(dynamic_cast<webots::Camera *>(device), this->shared_from_this());
        camera->enable(128);
        sensors_.push_back(camera);
        break;
      }
      case webots::Node::INERTIAL_UNIT:
      {
        auto imu = std::make_shared<wb_ros2_interface::sensors::WbRos2Imu>(dynamic_cast<webots::InertialUnit *>(device),
                                                                           this->shared_from_this());
        imu->enable(128);
        sensors_.push_back(imu);
        break;
      }
      case webots::Node::GPS:
      {
        auto gps = std::make_shared<wb_ros2_interface::sensors::WbRos2GPS>(dynamic_cast<webots::GPS *>(device),
                                                                           this->shared_from_this());
        sensors_.push_back(gps);
        break;
      }
      */
      }
    }
  }

  std::string WebotsNode::fixedNameString(const std::string &name)
  {
    std::string fixedName = name;
    std::replace(fixedName.begin(), fixedName.end(), '-', '_');
    std::replace(fixedName.begin(), fixedName.end(), '.', '_');
    std::replace(fixedName.begin(), fixedName.end(), ' ', '_');
    std::replace(fixedName.begin(), fixedName.end(), ')', '_');
    std::replace(fixedName.begin(), fixedName.end(), '(', '_');
    return fixedName;
  }

  void WebotsNode::timerCallback()
  {
    RCLCPP_INFO(get_logger(), "aaaa");
    mRobot->step(mStep);
    for (std::shared_ptr<PluginInterface> plugin : mPlugins)
      plugin->step(mStep);
  }

  void WebotsNode::registerPlugin(const std::string &pathToPlugin, const std::map<std::string, std::string> &arguments)
  {
    void *handle = dlopen(pathToPlugin.c_str(), RTLD_LAZY);
    if (!handle)
    {
      fprintf(stderr, "dlopen failure: %s\n", dlerror());
      exit(EXIT_FAILURE);
    }
    creatorFunction create = (creatorFunction)dlsym(handle, "create_plugin");

    // std::shared_ptr<PluginInterface> plugin = (*create)(this, {{"name", "ds0"}});
    // mPlugins.push_back(plugin);

    dlclose(handle);
  }

} // end namespace webots_ros2
