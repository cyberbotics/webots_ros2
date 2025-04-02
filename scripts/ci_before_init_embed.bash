#!/usr/bin/env bash

ROS_DISTRO=$1

export TEST_WITH_WEBOTS_NIGTHLY=1
export WEBOTS_RELEASE_VERSION=2025a
export WEBOTS_OFFSCREEN=1
export CI=1
export DEBIAN_FRONTEND=noninteractive
# The upstream workspace allows to install packages from sources for the tests (Turtlebot3 here)
if [[ "${ROS_DISTRO}" != "rolling" ]]; then
    export UPSTREAM_WORKSPACE=/root/turtlebot_ws/
fi


# TODO: Revert once the https://github.com/ros-planning/navigation2/issues/3033 issue is fixed.
# Fast-DDS is not working properly with the Nav2 package on Humble. Using Cyclone DDS instead.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
