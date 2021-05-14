cmake_minimum_required(VERSION 3.10)
project(webots_ros2_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

include_directories(
  include
  $ENV{WEBOTS_HOME}/include/controller/c
  $ENV{WEBOTS_HOME}/include/controller/cpp
)

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(webots_ros2_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_executable(driver 
  src/Driver.cpp
  src/WebotsNode.cpp
  src/devices/Ros2SensorPlugin.cpp
  src/devices/Ros2Lidar.cpp
  src/devices/Ros2Camera.cpp
  src/devices/Ros2GPS.cpp
  src/devices/Ros2RangeFinder.cpp
  src/devices/Ros2DistanceSensor.cpp
  src/devices/Ros2LightSensor.cpp
  src/utils/Math.cpp
  src/utils/Utils.cpp
)
ament_target_dependencies(driver 
  action_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  webots_ros2_msgs
  tinyxml2_vendor
  TinyXML2
)
target_link_libraries(driver 
  ${WEBOTS_LIB}  
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS driver
  RUNTIME 
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  webots_ros2_msgs
  tinyxml2_vendor
  TinyXML2
)

ament_package()
