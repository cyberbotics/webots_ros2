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

// wb_ros2_interface
#include <webots_ros2_cpp/wb_ros2_interface.hpp>

namespace wb_ros2_interface {

WbRos2Interface::WbRos2Interface() :
  Node("webots_ros2_interface"),
  step_(32)
{
}

WbRos2Interface::~WbRos2Interface() {
}

void WbRos2Interface::setup() {
  setupRobot();
  fixName();
  setupSensors();
  std::chrono::milliseconds ms(step_);
  timer_ = this->create_wall_timer(ms, 
    std::bind(&WbRos2Interface::timerCallback, this));
}

void WbRos2Interface::setupRobot() {
  robot_ = std::make_unique<webots::Supervisor>();
}

std::string WbRos2Interface::fixedNameString(const std::string &name) {
  std::string fixedName = name;
  std::replace(fixedName.begin(), fixedName.end(), '-', '_');
  std::replace(fixedName.begin(), fixedName.end(), '.', '_');
  std::replace(fixedName.begin(), fixedName.end(), ' ', '_');
  std::replace(fixedName.begin(), fixedName.end(), ')', '_');
  std::replace(fixedName.begin(), fixedName.end(), '(', '_');
  return fixedName;
}

void WbRos2Interface::fixName() {
  std::string webotsPID;
  std::string webotsHostname;
  std::ostringstream s;
  s << getppid();
  webotsPID = s.str();

  char hostname[256];
  gethostname(hostname, 256);
  webotsHostname = std::string(hostname);

  robot_name_ = robot_->getName();
  robot_name_ += '_' + webotsPID + '_' + webotsHostname;
  robot_name_ = fixedNameString(robot_name_);
}

void WbRos2Interface::setupSensors() {
  int number_of_devices = robot_->getNumberOfDevices();
  for (int i = 0; i < number_of_devices; i++) {
    auto temp_device = robot_->getDeviceByIndex(i);
    switch (temp_device->getNodeType()) {
      case webots::Node::LIDAR:
        auto lidar = std::make_shared<wb_ros2_interface::sensors::WbRos2Lidar>
          (dynamic_cast<webots::Lidar*>(temp_device), this->shared_from_this());
        lidar->enable(128);
        sensors_.push_back(lidar);
        break;
    }
  }
}

void WbRos2Interface::timerCallback() {
  robot_->step(step_);
  for (const auto& sensor : sensors_)
    sensor->publish();
}

} // end namespace wb_ros2_interface
