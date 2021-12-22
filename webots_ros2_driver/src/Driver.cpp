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


std::shared_ptr<webots_ros2_driver::WebotsNode> node;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  std::cout << "mySigIntHandler begin" << std::endl;

  node->test = true;
  while (node->inStep)
    std::cout << "wait for inStep" << std::endl;

  std::cout << "mySigIntHandler end" << std::endl;
  rclcpp::shutdown();
}

void mySigIntHandler2()
{
  std::cout << "mySigIntHandler begin" << std::endl;

  node->test = true;
  while (node->inStep)
    std::cout << "wait for inStep" << std::endl;

  std::cout << "mySigIntHandler end" << std::endl;
  rclcpp::shutdown();
}


int main(int argc, char **argv)
{
  signal(SIGINT, mySigIntHandler);
  signal(SIGTERM, mySigIntHandler);

  rclcpp::InitOptions options{};
  options.shutdown_on_sigint = false;
  rclcpp::init(argc, argv, options);



  if(rclcpp::uninstall_signal_handlers())
    std::cout << "uninstall_signal_handlers" << std::endl;

  //rclcpp::on_shutdown(mySigIntHandler2);

  //webots::Supervisor* robot;

  // Check if the robot can be a driver, if not create a simple Supervisor
  //if (webots::Driver::isInitialisationPossible())
  //  robot = new webots::Driver();
  //else
  //  robot = new webots::Supervisor();

  //std::string robotName = robot->getName();
  //for (char notAllowedChar : " -.)(")
  //  std::replace(robotName.begin(), robotName.end(), notAllowedChar, '_');

  //node = std::make_shared<webots_ros2_driver::WebotsNode>(robotName, robot);
  //node->init();

  auto node2 = rclcpp::Node::make_shared("test_prog");

  std::cout << "spin" << std::endl;

  rclcpp::spin(node2);
  std::cout << "will del robot" << std::endl;
  //delete robot;
  std::cout << "has del robot" << std::endl;
  rclcpp::shutdown();
  std::cout << "has shutdown" << std::endl;
  return 0;
}
