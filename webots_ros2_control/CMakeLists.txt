cmake_minimum_required(VERSION 3.5)
project(webots_ros2_control)

# Check which ROS distribution is used, ros2control depends of that
if($ENV{ROS_DISTRO} MATCHES "humble")
  add_compile_definitions(HUMBLE)
elseif($ENV{ROS_DISTRO} MATCHES "jazzy")
  add_compile_definitions(JAZZY)
elseif($ENV{ROS_DISTRO} MATCHES "kilted")
  add_compile_definitions(KILTED)
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

# macOS: Ros2Control.cpp calls libController C symbols (wb_robot_get_basic_time_step,
# wb_robot_get_time, ...) that webots_ros2_driver links but does NOT re-export; macOS does not
# chain DT_NEEDED, so these targets must link the Webots CppController library themselves or the
# final link fails with "Undefined symbols: _wb_robot_get_*". webots_ros2_driver has already
# resolved/fetched Webots by now, so just locate that WEBOTS_HOME (env, /Applications, or the
# driver's ~/.cache/webots/<ver> download) and collect the controller dylibs.
# Accept either WEBOTS_HOME convention. Webots' own webots-controller binary takes the
# .app ROOT and appends the rest itself -- "DYLD_LIBRARY_PATH=%s/Contents/lib/controller"
# is a literal string inside that binary -- and that is the value the Webots docs tell
# users to export, so /Applications/Webots.app is what people actually have set. The CMake
# below wants the Contents dir, so normalize instead of rejecting: testing only
# "$ENV{WEBOTS_HOME}/lib/controller" makes a correctly-set WEBOTS_HOME fail the check and
# fall through to an unnecessary download (or a hard error offline).
macro(_webots_resolve_home _out)
  set(${_out} "")
  if(DEFINED ENV{WEBOTS_HOME})
    if(EXISTS "$ENV{WEBOTS_HOME}/lib/controller")
      set(${_out} "$ENV{WEBOTS_HOME}")                  # already .../Webots.app/Contents
    elseif(EXISTS "$ENV{WEBOTS_HOME}/Contents/lib/controller")
      set(${_out} "$ENV{WEBOTS_HOME}/Contents")         # the .app root
    else()
      message(WARNING "WEBOTS_HOME is set to \"$ENV{WEBOTS_HOME}\" but neither "
        "lib/controller nor Contents/lib/controller exists under it; ignoring it.")
    endif()
  endif()
endmacro()

if(APPLE)
  # Must match webots_ros2_driver's pinned release. The driver links the controller
  # libraries of exactly this version, and if the plugins here link a different one the two
  # halves disagree on the controller ABI at runtime. Picking "whatever is cached" is wrong
  # as soon as two releases are cached.
  set(WEBOTS_VERSION "R2025a")
  _webots_resolve_home(_wbc_home)
  if(_wbc_home)
    # taken from the environment, already normalized
  elseif(EXISTS /Applications/Webots.app/Contents/lib/controller)
    set(_wbc_home /Applications/Webots.app/Contents)
  else()
    if(DEFINED ENV{WEBOTS_CACHE_DIR})
      set(_wbc_cache "$ENV{WEBOTS_CACHE_DIR}")
    else()
      set(_wbc_cache "$ENV{HOME}/.cache/webots")
    endif()
    if(EXISTS "${_wbc_cache}/${WEBOTS_VERSION}/Webots.app/Contents/lib/controller")
      set(_wbc_home "${_wbc_cache}/${WEBOTS_VERSION}/Webots.app/Contents")
    else()
      # Only as a last resort, and say so: this may not be the driver's release.
      file(GLOB _wbc_apps "${_wbc_cache}/*/Webots.app/Contents")
      foreach(_a ${_wbc_apps})
        if(EXISTS "${_a}/lib/controller")
          set(_wbc_home "${_a}")
        endif()
      endforeach()
      if(_wbc_home)
        message(WARNING "webots_ros2_control: Webots ${WEBOTS_VERSION} is not cached; "
          "falling back to ${_wbc_home}, which may not match webots_ros2_driver's release.")
      endif()
    endif()
  endif()
  if(_wbc_home)
    file(GLOB WEBOTS_CTRL_LIB
      "${_wbc_home}/lib/controller/*CppController${CMAKE_SHARED_LIBRARY_SUFFIX}"
      "${_wbc_home}/lib/controller/*Controller${CMAKE_SHARED_LIBRARY_SUFFIX}")
  else()
    message(WARNING "webots_ros2_control: could not locate Webots lib/controller; wb_robot_* may be unresolved.")
  endif()
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
  ${webots_ros2_driver_INCLUDE_DIRS}
)
if ($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "jazzy")
  ament_target_dependencies(
    ${PROJECT_NAME}
    hardware_interface
    controller_manager
    pluginlib
    rclcpp
    webots_ros2_driver
  )
else()
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
    hardware_interface::hardware_interface
    controller_manager::controller_manager
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${WEBOTS_CTRL_LIB}
  )
endif()

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
  ${webots_ros2_driver_INCLUDE_DIRS}
)
if ($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "jazzy")
  ament_target_dependencies(
    ${PROJECT_NAME}_system
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    webots_ros2_driver
  )
else()
  target_link_libraries(${PROJECT_NAME}_system
    PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${WEBOTS_CTRL_LIB}
  )
endif()

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
