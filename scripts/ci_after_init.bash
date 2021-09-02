#!/usr/bin/env bash

apt update
apt install -y wget dialog apt-utils psmisc
wget https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb -O /tmp/webots.deb
apt install -y /tmp/webots.deb xvfb

if [${ROS_DISTRO} = "foxy"] then
    apt install ros-foxy-turtlebot3-cartographer
    apt install ros-foxy-turtlebot3-navigation2
fi