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

#include <fcntl.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/PythonPlugin.hpp>
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

  rclcpp::NodeOptions nodeOptions;
  nodeOptions.arguments(std::vector<std::string>(argv + 1, argv + argc));
  std::shared_ptr<webots_ros2_driver::WebotsNode> node =
    std::make_shared<webots_ros2_driver::WebotsNode>(robotName, nodeOptions);
  node->init();

  // The parent process must be ros2 run. Declaring from launch file is deprecated. Remove with 2024.0.0.
  const pid_t parentPid = getppid();
  char buffer[4096];
  snprintf(buffer, sizeof(buffer), "/proc/%d/cmdline", parentPid);
  const int fd = open(buffer, O_RDONLY);
  if (fd == -1) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open cmdline file.");
    return -1;
  }
  const int size = read(fd, buffer, sizeof(buffer));
  close(fd);
  for (int i = 0; i < size; i++) {
    if (!buffer[i])
      buffer[i] = ' ';
  }
  buffer[size] = '\0';
  if (strstr(buffer, "ros2 launch"))
    RCLCPP_WARN(node->get_logger(), "\033[33mThe direct declaration of the driver node in the launch file is deprecated. "
                                    "Please use the new WebotsController node instead.\033[0m");

  RCLCPP_INFO(node->get_logger(), "Controller successfully connected to robot in Webots simulation.");
  while (true) {
    if (node->step() == -1)
      break;
    rclcpp::spin_some(node);
  }
  // Check if the plugin is actually an instance of PythonPlugin
  bool isPython = false;
  for (std::shared_ptr<webots_ros2_driver::PluginInterface> plugin : node->mPlugins) {
    std::shared_ptr<webots_ros2_driver::PythonPlugin> pythonPlugin =
      std::dynamic_pointer_cast<webots_ros2_driver::PythonPlugin>(plugin);
    if (pythonPlugin) {
      pythonPlugin->stop();  // stop only for python plugins
      isPython = true;
      break;
    }
  }
  if (!isPython) {
    wb_robot_cleanup();
  }
  RCLCPP_INFO(node->get_logger(), "Controller successfully disconnected from robot in Webots simulation.");
  rclcpp::shutdown();
  return 0;
}
