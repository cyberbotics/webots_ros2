#!/usr/bin/env bash

ROS_DISTRO=$1

export WEBOTS_RELEASE_VERSION=2023b
export WEBOTS_OFFSCREEN=1
export CI=1
export DEBIAN_FRONTEND=noninteractive

# TODO: Revert once the https://github.com/ros-planning/navigation2/issues/3033 issue is fixed.
# Fast-DDS is not working properly with the Nav2 package on Humble. Using Cyclone DDS instead.
if [[ "${ROS_DISTRO}" == "humble" ]]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
