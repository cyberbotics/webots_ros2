cmake_minimum_required(VERSION 3.10)
project(webots_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

# Check which ROS distribution is used, vision_msgs depends of that
if($ENV{ROS_DISTRO} MATCHES "humble")
  add_compile_definitions(HUMBLE)
elseif($ENV{ROS_DISTRO} MATCHES "jazzy")
  add_compile_definitions(JAZZY)
elseif($ENV{ROS_DISTRO} MATCHES "rolling")
  add_compile_definitions(ROLLING)
endif()

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(webots_ros2_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(yaml-cpp REQUIRED)

if($ENV{ROS_DISTRO} MATCHES "humble")
  find_package(Python 3.10 EXACT REQUIRED COMPONENTS Development)
elseif($ENV{ROS_DISTRO} MATCHES "jazzy")
  find_package(Python 3.12 EXACT REQUIRED COMPONENTS Development)
elseif($ENV{ROS_DISTRO} MATCHES "rolling")
  find_package(Python 3.12 EXACT REQUIRED COMPONENTS Development)
endif()

add_custom_target(compile-lib-controller ALL
  COMMAND ${CMAKE_COMMAND} -E env "WEBOTS_HOME=${CMAKE_CURRENT_SOURCE_DIR}/webots" make release -f Makefile > /dev/null 2>&1
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webots/src/controller
)

add_custom_target(compile-lib-vehicle ALL
  COMMAND ${CMAKE_COMMAND} -E env "WEBOTS_HOME=${CMAKE_CURRENT_SOURCE_DIR}/webots" make release -f Makefile > /dev/null 2>&1
  DEPENDS compile-lib-controller
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webots/projects/default/libraries/vehicle
)

add_custom_target(compile-generic-window ALL
  COMMAND ${CMAKE_COMMAND} -E env "WEBOTS_HOME=${CMAKE_CURRENT_SOURCE_DIR}/webots" make release -f Makefile > /dev/null 2>&1
  DEPENDS compile-lib-controller
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webots/resources/projects/libraries/generic_robot_window
)

set(WEBOTS_LIB_BASE webots/lib/controller)

include_directories(
  include
  webots/include/controller/c
  webots/include/controller/cpp
  ${Python_INCLUDE_DIRS}
)

link_directories(${WEBOTS_LIB_BASE})

set(WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppCar${CMAKE_SHARED_LIBRARY_SUFFIX}
)

ament_python_install_package(controller
  PACKAGE_DIR ${WEBOTS_LIB_BASE}/python/controller)

ament_python_install_package(vehicle
  PACKAGE_DIR ${WEBOTS_LIB_BASE}/python/vehicle)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME})

# Driver
set(CMAKE_INSTALL_RPATH "$ORIGIN/../controller") # Deprecated with new WebotsController node, remove with 2024.0.0
add_executable(driver
  src/Driver.cpp
  src/WebotsNode.cpp
  src/PythonPlugin.cpp
  src/plugins/Ros2SensorPlugin.cpp
  src/plugins/static/Ros2Lidar.cpp
  src/plugins/static/Ros2LED.cpp
  src/plugins/static/Ros2Pen.cpp
  src/plugins/static/Ros2Camera.cpp
  src/plugins/static/Ros2GPS.cpp
  src/plugins/static/Ros2RangeFinder.cpp
  src/plugins/static/Ros2DistanceSensor.cpp
  src/plugins/static/Ros2LightSensor.cpp
  src/plugins/static/Ros2Emitter.cpp
  src/plugins/static/Ros2Receiver.cpp
  src/plugins/static/Ros2Compass.cpp
  src/plugins/static/Ros2VacuumGripper.cpp
  src/utils/Math.cpp
  src/utils/Utils.cpp
)
ament_target_dependencies(driver
  rosgraph_msgs
  rclcpp
  pluginlib
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
  webots_ros2_msgs
  tinyxml2_vendor
  TinyXML2
)
add_dependencies(driver
  compile-lib-vehicle
)
target_link_libraries(driver
  ${WEBOTS_LIB}
  ${Python_LIBRARIES}
  yaml-cpp
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Dynamic IMU
set(CMAKE_INSTALL_RPATH "$ORIGIN/controller") # Deprecated with new WebotsController node, remove with 2024.0.0
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
  webots_ros2_msgs
  pluginlib
  tf2_ros
)
add_dependencies(${PROJECT_NAME}_imu
  compile-lib-vehicle
)
target_link_libraries(${PROJECT_NAME}_imu
  ${WEBOTS_LIB}
)
install(TARGETS ${PROJECT_NAME}_imu
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Dynamic RGBD
set(CMAKE_INSTALL_RPATH "$ORIGIN/controller") # Deprecated with new WebotsController node, remove with 2024.0.0
add_library(
  ${PROJECT_NAME}_rgbd
  SHARED
  src/plugins/dynamic/Ros2RGBD.cpp
  src/plugins/Ros2SensorPlugin.cpp
  src/utils/Utils.cpp
)
ament_target_dependencies(${PROJECT_NAME}_rgbd
  rclcpp
  sensor_msgs
  pluginlib
  tf2_ros
)
add_dependencies(${PROJECT_NAME}_rgbd
  compile-lib-vehicle
)
target_link_libraries(${PROJECT_NAME}_rgbd
  ${WEBOTS_LIB}
)
install(TARGETS ${PROJECT_NAME}_rgbd
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# libController
install(
  DIRECTORY webots/include/controller/c
  DESTINATION include/webots
)
install(
  DIRECTORY webots/include/controller/cpp
  DESTINATION include/webots
)
install(
  DIRECTORY ${WEBOTS_LIB_BASE}/
  DESTINATION lib/controller
  PATTERN "python*" EXCLUDE
  PATTERN "matlab" EXCLUDE
  PATTERN "*Controller*"
  PATTERN "*CppController*"
  PATTERN "*car*"
  PATTERN "*CppCar*"
  PATTERN "*driver*"
  PATTERN "*CppDriver*"
  PATTERN "*generic_robot_window*"
)

# Prevent pluginlib from using boost
target_compile_definitions(driver PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME}_imu PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME}_rgbd PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(${PROJECT_NAME} webots_ros2_imu.xml)
pluginlib_export_plugin_description_file(${PROJECT_NAME} webots_ros2_rgbd.xml)

# Install ROS 2 nodes
install(PROGRAMS ${PROJECT_NAME}/ros2_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install webots-controller script
install(
  PROGRAMS webots/webots-controller
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Ament export
set(WEBOTS_LIB_PATH
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX}
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}CppCar${CMAKE_SHARED_LIBRARY_SUFFIX}
  controller/${CMAKE_SHARED_LIBRARY_PREFIX}generic_robot_window${CMAKE_SHARED_LIBRARY_SUFFIX}
)

ament_export_include_directories(
  include
  include/webots/c
  include/webots/cpp
)
ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
  webots_ros2_msgs
  tinyxml2_vendor
  TinyXML2
)
ament_export_libraries(
  ${PROJECT_NAME}_imu
  ${PROJECT_NAME}_rgbd
  ${WEBOTS_LIB_PATH}
)
ament_package()
