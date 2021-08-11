cmake_minimum_required(VERSION 3.10)
project(pytest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PythonLibs 3.8 EXACT REQUIRED)

include_directories(
  /home/lukic/webots2/include/controller/c
  /home/lukic/webots2/include/controller/cpp
)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(/home/lukic/webots2/lib/controller)

add_executable(pytest main.cpp)

target_link_libraries(pytest
    ${PYTHON_LIBRARIES}
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)
