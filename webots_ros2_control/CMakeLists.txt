cmake_minimum_required(VERSION 3.5)
project(webots_ros2_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)


# ros2_control
# ros2_control: compile
add_library(
  ${PROJECT_NAME}_control
  SHARED
  src/Ros2Control.cpp
)
target_include_directories(
  ${PROJECT_NAME}_control
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}_control
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface wb_ros2_control.xml)

# ros2_control: install
install(
  TARGETS ${PROJECT_NAME}_control
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# ros2_control: export
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_control
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
