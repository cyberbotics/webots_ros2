cmake_minimum_required(VERSION 3.5)
project(webots_ros2_control)

# Check which ROS distribution is used, ros2control depends of that
if($ENV{ROS_DISTRO} MATCHES "humble")
  add_compile_definitions(HUMBLE)
elseif($ENV{ROS_DISTRO} MATCHES "jazzy")
  add_compile_definitions(JAZZY)
elseif($ENV{ROS_DISTRO} MATCHES "rolling")
  add_compile_definitions(ROLLING)
endif()

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
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(webots_ros2_driver REQUIRED)

add_compile_definitions(HARDWARE_INTERFACE_VERSION_MAJOR=${hardware_interface_VERSION_MAJOR})
add_compile_definitions(HARDWARE_INTERFACE_VERSION_MINOR=${hardware_interface_VERSION_MINOR})

if(MSVC OR MSYS OR MINGW OR WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# webots_ros2 Plugin
add_library(
  ${PROJECT_NAME}
  SHARED
  src/Ros2Control.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  webots_ros2_driver
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(webots_ros2_driver webots_ros2_control.xml)

# ros2_control System
add_library(
  ${PROJECT_NAME}_system
  SHARED
  src/Ros2ControlSystem.cpp
)
target_include_directories(
  ${PROJECT_NAME}_system
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  webots_ros2_driver
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_system PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(${PROJECT_NAME} webots_ros2_control_system.xml)

# Common
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  webots_ros2_driver
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_system
)

ament_package()
