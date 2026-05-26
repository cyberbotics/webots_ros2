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
elseif($ENV{ROS_DISTRO} MATCHES "kilted")
  add_compile_definitions(KILTED)
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
find_package(yaml_cpp_vendor REQUIRED)

if($ENV{ROS_DISTRO} MATCHES "humble|iron")
    set(PYTHON_VER 3.10)
else()
    set(PYTHON_VER 3.12)
endif()
find_package(Python ${PYTHON_VER} EXACT REQUIRED COMPONENTS Development)


# Configure Webots library paths:
# - Use system-installed Webots.app bundle on macOS
# - Otherwise assume in-tree source build (Linux or macOS)

if(APPLE AND EXISTS /Applications/Webots.app)
  set(WEBOTS_PREBUILT TRUE)
  set(WEBOTS_HOME /Applications/Webots.app/Contents)
  include_directories(
    ${WEBOTS_HOME}/include/controller/c
    ${WEBOTS_HOME}/include/controller/cpp
    ${Python_INCLUDE_DIRS}
  )
  link_directories(${WEBOTS_HOME}/lib/controller)
  set(WEBOTS_LIB_BASE ${WEBOTS_HOME}/lib/controller)
  set(WEBOTS_CONTROLLER_EXEC ${WEBOTS_HOME}/MacOS/webots-controller)
else()
  set(WEBOTS_PREBUILT FALSE)
  set(WEBOTS_LIB_BASE ${CMAKE_CURRENT_SOURCE_DIR}/webots/lib/controller)
  include_directories(
    include
    webots/include/controller/c
    webots/include/controller/cpp
    ${Python_INCLUDE_DIRS}
  )

  # Build Webots libraries from source
  add_custom_target(compile-lib-controller ALL
    COMMAND ${CMAKE_COMMAND} -E env "WEBOTS_HOME=${CMAKE_CURRENT_SOURCE_DIR}/webots" make release -f Makefile 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webots/src/controller
  )

  add_custom_target(compile-lib-vehicle ALL
    COMMAND ${CMAKE_COMMAND} -E env "WEBOTS_HOME=${CMAKE_CURRENT_SOURCE_DIR}/webots" make release -f Makefile 
    DEPENDS compile-lib-controller
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webots/projects/default/libraries/vehicle
  )

  add_custom_target(compile-generic-window ALL
    COMMAND ${CMAKE_COMMAND} -E env "WEBOTS_HOME=${CMAKE_CURRENT_SOURCE_DIR}/webots" make release -f Makefile 
    DEPENDS compile-lib-controller
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webots/resources/projects/libraries/generic_robot_window
  )
  set(WEBOTS_CONTROLLER_EXEC webots/webots-controller)
endif()

file(GLOB WEBOTS_LIB_FILES
  "${WEBOTS_LIB_BASE}/*Controller${CMAKE_SHARED_LIBRARY_SUFFIX}"
  "${WEBOTS_LIB_BASE}/*CppController${CMAKE_SHARED_LIBRARY_SUFFIX}"
  "${WEBOTS_LIB_BASE}/*driver${CMAKE_SHARED_LIBRARY_SUFFIX}"
  "${WEBOTS_LIB_BASE}/*CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX}"
  "${WEBOTS_LIB_BASE}/*car${CMAKE_SHARED_LIBRARY_SUFFIX}"
  "${WEBOTS_LIB_BASE}/*CppCar${CMAKE_SHARED_LIBRARY_SUFFIX}"
)
set(WEBOTS_LIB ${WEBOTS_LIB_FILES})

# Only install Python packages if the source directories exist
foreach(pkg IN ITEMS controller vehicle)
  if(EXISTS ${WEBOTS_LIB_BASE}/python/${pkg})
    ament_python_install_package(${pkg} PACKAGE_DIR ${WEBOTS_LIB_BASE}/python/${pkg})
  endif()
endforeach()

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

if ($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "jazzy")
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
  target_link_libraries(driver
    ${WEBOTS_LIB}
    ${Python_LIBRARIES}
    yaml-cpp
  )
else()
  target_link_libraries(driver
    ${rosgraph_msgs_TARGETS}
    rclcpp::rclcpp
    pluginlib::pluginlib
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    ${vision_msgs_TARGETS}
    ${webots_ros2_msgs_TARGETS}
    tinyxml2::tinyxml2
    ${WEBOTS_LIB}
    ${Python_LIBRARIES}
    yaml-cpp
  )
endif()

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
if ($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "jazzy")
  ament_target_dependencies(${PROJECT_NAME}_imu
    rclcpp
    sensor_msgs
    webots_ros2_msgs
    pluginlib
    tf2_ros
  )
  target_link_libraries(${PROJECT_NAME}_imu
    ${WEBOTS_LIB}
  )
else()
  target_link_libraries(${PROJECT_NAME}_imu
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${webots_ros2_msgs_TARGETS}
    pluginlib::pluginlib
    tf2_ros::tf2_ros
    ${WEBOTS_LIB}
  )
endif()

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
if ($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "jazzy")
  ament_target_dependencies(${PROJECT_NAME}_rgbd
    rclcpp
    sensor_msgs
    pluginlib
    tf2_ros
  )
  target_link_libraries(${PROJECT_NAME}_rgbd
    ${WEBOTS_LIB}
  )
else()
  target_link_libraries(${PROJECT_NAME}_rgbd
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    pluginlib::pluginlib
    tf2_ros::tf2_ros
    ${WEBOTS_LIB}
  )
endif()

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

# Install scripts
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install ROS 2 nodes
install(PROGRAMS ${PROJECT_NAME}/ros2_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Dynamic IMU and RGBD libraries: similar cross-platform treatment applies
# <<< CROSS-PLATFORM CHANGE: add_dependencies only if source build
if(NOT WEBOTS_PREBUILT)
  add_dependencies(driver compile-lib-vehicle)
  add_dependencies(${PROJECT_NAME}_imu compile-lib-vehicle)
  add_dependencies(${PROJECT_NAME}_rgbd compile-lib-vehicle)
endif()

install(PROGRAMS ${WEBOTS_CONTROLLER_EXEC}
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
