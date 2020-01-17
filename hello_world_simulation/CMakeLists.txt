cmake_minimum_required(VERSION 3.5)

project(hello_world_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
