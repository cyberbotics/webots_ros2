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

// webots_ros2_cpp
#include <webots_ros2_cpp/wb_ros2_interface.hpp>

// std
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>

std::shared_ptr<robot_state_publisher::RobotStatePublisher> 
createRobotPublisher(
  const std::shared_ptr<wb_ros2_interface::WbRos2Interface>& node) 
{
  auto urdf = node->getURDF();
  rclcpp::NodeOptions options;
  options.append_parameter_override("robot_description", urdf);
  return std::make_shared<
    robot_state_publisher::RobotStatePublisher>(options);
}

int main(int argc, char **argv) {
  // Initialize without sigint handler
  rclcpp::init(argc, argv);

  // Start an asyncronous spinner
  auto webots_node = std::make_shared<wb_ros2_interface::WbRos2Interface>();
  webots_node->setup();
  auto robot_publisher = createRobotPublisher(webots_node);
  
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(webots_node);
  exec.add_node(robot_publisher);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}