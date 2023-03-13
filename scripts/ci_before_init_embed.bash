#!/usr/bin/env bash

ROS_DISTRO=$1

export WEBOTS_RELEASE_VERSION=2023a-rev1
export WEBOTS_OFFSCREEN=1
export CI=1
export DEBIAN_FRONTEND=noninteractive
if [[ "${ROS_DISTRO}" == "humble" ]]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
