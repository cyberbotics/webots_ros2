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

#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

int main(int argc, char **argv) {
  // Check if the robot can be a driver, if not create a simple Supervisor
  if (wbu_driver_initialization_is_possible())
    wbu_driver_init();
  else
    wb_robot_init();

  // Let WebotsNode handle the system signals
  webots_ros2_driver::WebotsNode::handleSignals();

  rclcpp::InitOptions options{};
  options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, options);

  std::string robotName(wb_robot_get_name());
  for (char notAllowedChar : " -.)(")
    std::replace(robotName.begin(), robotName.end(), notAllowedChar, '_');

  std::shared_ptr<webots_ros2_driver::WebotsNode> node = std::make_shared<webots_ros2_driver::WebotsNode>(robotName);
  node->init();
  RCLCPP_INFO(node->get_logger(), "Controller successfully connected to robot in Webots simulation.");
  while (true) {
    if (node->step() == -1)
      break;
    rclcpp::spin_some(node);
  }
  wb_robot_cleanup();
  rclcpp::shutdown();
  return 0;
}
