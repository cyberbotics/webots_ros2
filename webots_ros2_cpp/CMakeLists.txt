cmake_minimum_required(VERSION 3.10)
project(webots_ros2_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g)

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
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(webots_ros2_msgs REQUIRED)

set(${PROJECT_NAME}_dependencies
  action_msgs  
  rclcpp
  robot_state_publisher
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  webots_ros2_msgs
)

#############
##  Build  ##
#############

add_library(${PROJECT_NAME} SHARED
  src/wb_ros2_interface.cc
  src/sensors/wb_ros2_camera.cc
  src/sensors/wb_ros2_gps.cc
  src/sensors/wb_ros2_imu.cc
  src/sensors/wb_ros2_joint_states.cc
  src/sensors/wb_ros2_lidar.cc
  src/math.cc
)
ament_target_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_dependencies}
)
target_link_libraries(${PROJECT_NAME} 
  ${WEBOTS_LIB}
)

add_executable(wb_task_node 
  src/nodes/wb_ros2_task.cc
)
ament_target_dependencies(wb_task_node 
  ${${PROJECT_NAME}_dependencies}
)
target_link_libraries(wb_task_node 
  ${PROJECT_NAME}
  ${WEBOTS_LIB}  
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS wb_task_node
  RUNTIME 
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME} 
)
ament_package()
