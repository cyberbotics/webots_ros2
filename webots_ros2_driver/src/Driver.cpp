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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots/vehicle/Driver.hpp>

/*
bool SIGINTReceived = false;

void customSigIntHandler(int sig)
{
  SIGINTReceived = true;
}
*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);//only test, to remove

  webots::Supervisor* robot;

  // Check if the robot can be a driver, if not create a simple Supervisor
  if (webots::Driver::isInitialisationPossible())
    robot = new webots::Driver();
  else
    robot = new webots::Supervisor();

  // Replace the signal handler for the WebotsNode and the robot by a custom one
  //signal(SIGINT, customSigIntHandler);
  //rclcpp::InitOptions options{};
  //options.shutdown_on_sigint = false;
  //rclcpp::init(argc, argv, options);

  std::string robotName = robot->getName();
  for (char notAllowedChar : " -.)(")
    std::replace(robotName.begin(), robotName.end(), notAllowedChar, '_');

  std::shared_ptr<webots_ros2_driver::WebotsNode> node = std::make_shared<webots_ros2_driver::WebotsNode>(robotName, robot);
  node->init();

  rclcpp::spin(node);
  delete robot;
  rclcpp::shutdown();
  return 0;
}
