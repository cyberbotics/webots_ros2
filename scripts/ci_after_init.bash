#!/usr/bin/env bash

ROS_DISTRO=$1

apt update
apt install -y wget dialog apt-utils psmisc
wget https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb -O /tmp/webots.deb
apt install -y /tmp/webots.deb xvfb

if [ "${ROS_DISTRO}" = "foxy" ]; then
    #apt install -y ros-foxy-turtlebot3-cartographer ros-foxy-turtlebot3-navigation2
fi
