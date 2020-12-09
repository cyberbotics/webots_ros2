cmake_minimum_required(VERSION 3.5)
project(webots_ros2_cpp)


# Compiler
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Webots
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(
  WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(
  $ENV{WEBOTS_HOME}/include/controller/c
  $ENV{WEBOTS_HOME}/include/controller/cpp
)


# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(driver src/driver.cpp)
target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(driver
  ${WEBOTS_LIB}
)
ament_target_dependencies(driver rclcpp sensor_msgs)
install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})


# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
