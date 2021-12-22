#!/usr/bin/env bash

ROS_DISTRO=$1
ROS_REPO=$2

apt update
apt install -y wget dialog apt-utils psmisc
wget https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb -O /tmp/webots.deb
apt install -y /tmp/webots.deb xvfb

# The following packages are only available in the ROS 2 Foxy distribution. Therefore, we cannot include them in the package.xml, but we have to install them manually here.
if [ "${ROS_DISTRO}" = "foxy" ]; then
    apt install -y ros-foxy-turtlebot3-cartographer ros-foxy-turtlebot3-navigation2
fi
