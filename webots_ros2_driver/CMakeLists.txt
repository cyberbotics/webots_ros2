cmake_minimum_required(VERSION 3.10)
project(webots_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(webots_ros2_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(PythonLibs 3.8 EXACT REQUIRED)

if (UNIX AND NOT APPLE)
  set(WEBOTS_LIB_BASE webots/lib/linux-gnu)
endif ()
if (APPLE)
  set(WEBOTS_LIB_BASE webots/lib/darwin19)
endif ()
if (MSVC OR MSYS OR MINGW OR WIN32)
  set(WEBOTS_LIB_BASE webots/lib/msys)
endif ()

include_directories(
  include
  webots/include/c
  webots/include/cpp
  ${PYTHON_INCLUDE_DIRS}
)

link_directories(${WEBOTS_LIB_BASE})

if (MSVC OR MSYS OR MINGW OR WIN32)
  # Windows requires the libController C++ part to be compiled.
  # See more here: https://cyberbotics.com/doc/guide/using-your-ide#visual-studio

  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  file(GLOB CppController_SRC CONFIGURE_DEPENDS "webots/source/cpp/*.cpp")
  file(GLOB CppCarDriver_SRC CONFIGURE_DEPENDS "webots/source/cpp/vehicle/*.cpp")
  add_library(
    CppControllerRos
    SHARED
    ${CppController_SRC}
    ${CppCarDriver_SRC}
  )
  target_link_libraries(CppControllerRos
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}
  )
  install(TARGETS CppControllerRos
    LIBRARY DESTINATION lib
  )
  set(WEBOTS_LIB
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}
    CppControllerRos
  )
  ament_export_libraries(CppControllerRos)
else()
  set(WEBOTS_LIB
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppCar${CMAKE_SHARED_LIBRARY_SUFFIX}
  )
endif()

ament_python_install_package(${PROJECT_NAME}_webots
  PACKAGE_DIR ${WEBOTS_LIB_BASE}/python38)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME})

add_executable(driver
  src/Driver.cpp
  src/WebotsNode.cpp
  src/PythonPlugin.cpp
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
  ${PYTHON_LIBRARIES}
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
  DIRECTORY webots/include/c
  DESTINATION include/webots
)
install(
  DIRECTORY webots/include/cpp
  DESTINATION include/webots
)
install(TARGETS ${PROJECT_NAME}_imu
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
if (MSVC OR MSYS OR MINGW OR WIN32)
  # Windows requires the library to be placed with executable
  install(
    FILES "${WEBOTS_LIB_BASE}/${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    FILES "${WEBOTS_LIB_BASE}/${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    FILES "${WEBOTS_LIB_BASE}/${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION lib/${PROJECT_NAME}
  )

  # Windows requires the C++ library to be placed with the Python module
  install(
    FILES "${WEBOTS_LIB_BASE}/${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}_webots/"
  )
  install(
    FILES "${WEBOTS_LIB_BASE}/${CMAKE_SHARED_LIBRARY_PREFIX}CppCar${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}_webots/"
  )
  install(
    FILES "${WEBOTS_LIB_BASE}/${CMAKE_SHARED_LIBRARY_PREFIX}CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}_webots/"
  )
else()
  install(
    DIRECTORY ${WEBOTS_LIB_BASE}/
    DESTINATION lib
    PATTERN "python*" EXCLUDE
    PATTERN "*Controller*"
    PATTERN "*CppController*"
    PATTERN "*car*"
    PATTERN "*CppCar*"
    PATTERN "*driver*"
    PATTERN "*CppDriver*"
  )
endif()

# Prevent pluginlib from using boost
target_compile_definitions(driver PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME}_imu PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(${PROJECT_NAME} webots_ros2_imu.xml)

# Ament export
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
  tf2_ros
  vision_msgs
  webots_ros2_msgs
  tinyxml2_vendor
  TinyXML2
)
ament_export_libraries(
  ${PROJECT_NAME}_imu
  ${WEBOTS_LIB}
)
ament_package()
