cmake_minimum_required(VERSION 3.10)
project(webots_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (UNIX)
  set(WEBOTS_LIB_BASE webots/lib/linux-gnu)
endif (UNIX)
if (WIN32)
  set(WEBOTS_LIB_BASE webots/lib/msys)
endif (WIN32)
if (MSVC)
  set(WEBOTS_LIB_BASE webots/lib/darwin19)
endif (MSVC)
link_directories(${WEBOTS_LIB_BASE})
set(WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)
include_directories(
  include
  webots/include/c
  webots/include/cpp
)

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
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
  src/plugins/Ros2SensorPlugin.cpp
  src/plugins/static/Ros2Lidar.cpp
  src/plugins/static/Ros2LED.cpp
  src/plugins/static/Ros2Camera.cpp
  src/plugins/static/Ros2GPS.cpp
  src/plugins/static/Ros2RangeFinder.cpp
  src/plugins/static/Ros2DistanceSensor.cpp
  src/plugins/static/Ros2LightSensor.cpp
  src/utils/Math.cpp
  src/utils/Utils.cpp
)

# Driver
ament_target_dependencies(driver 
  rosgraph_msgs
  rclcpp
  pluginlib
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

# Dynamic IMU
add_library(
  ${PROJECT_NAME}_imu
  SHARED
  src/plugins/dynamic/Ros2IMU.cpp
  src/plugins/Ros2SensorPlugin.cpp
  src/utils/Utils.cpp
)
ament_target_dependencies(${PROJECT_NAME}_imu 
  rclcpp
  sensor_msgs
  pluginlib
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}_imu 
  ${WEBOTS_LIB}  
)

install(
 DIRECTORY include/${PROJECT_NAME}
 DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY webots/include/c
  DESTINATION include/webots
)
install(
  DIRECTORY webots/include/cpp
  DESTINATION include/webots
)
install(TARGETS ${PROJECT_NAME}_imu
  RUNTIME 
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY ${WEBOTS_LIB_BASE}/
  DESTINATION lib/
)

# Prevent pluginlib from using boost
target_compile_definitions(driver PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME}_imu PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(${PROJECT_NAME} webots_ros2_imu.xml)

# Ament export
ament_export_include_directories(include)
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
