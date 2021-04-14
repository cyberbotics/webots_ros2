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

int main(int argc, char **argv) {
  // Initialize without sigint handler
  rclcpp::init(argc, argv);

  // Start an asyncronous spinner
  auto node = std::make_shared<wb_ros2_interface::WbRos2Interface>();
  node->setup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}