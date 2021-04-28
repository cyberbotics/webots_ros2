cmake_minimum_required(VERSION 3.10)
project(webots_ros2_distance_sensor_plugin)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_cpp REQUIRED)

set(${PROJECT_NAME}_dependencies
  rclcpp
  webots_ros2_cpp
)

add_library(${PROJECT_NAME} SHARED
    DistanceSensorPlugin.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_dependencies}
)
target_link_libraries(${PROJECT_NAME} 
  ${WEBOTS_LIB}
)
