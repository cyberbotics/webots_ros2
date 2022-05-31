#!/usr/bin/env bash

ROS_DISTRO=$1
ROS_REPO=$2

# Take the latest nightly build
YESTERDAY_WEEK_DAY_NUMBER=`date --date="1 day ago" +"%u"`
LAST_NIGHTLY_DAY_OLD=1
# There is no nightly build the weekend
if [ ${YESTERDAY_WEEK_DAY_NUMBER} -gt 5 ]; then
    LAST_NIGHTLY_DAY_OLD="$((${YESTERDAY_WEEK_DAY_NUMBER}-4))"
fi
NIGHTLY_DATE=`date --date="${LAST_NIGHTLY_DAY_OLD} day ago" +"%-d_%-m_%Y"`
WEBOTS_NIGHTLY_VERSION="nightly_${NIGHTLY_DATE}"

apt update
apt install -y wget dialog apt-utils psmisc lsb-release
wget https://github.com/cyberbotics/webots/releases/download/${WEBOTS_NIGHTLY_VERSION}/webots_${WEBOTS_RELEASE_VERSION}_amd64.deb -O /tmp/webots.deb
apt install -y /tmp/webots.deb xvfb

# OpenSSL patch for ubuntu 22
UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSION == "22.04" ]]; then
  wget https://cyberbotics.com/files/repository/dependencies/linux64/release/libssl_1.1.tar.xz
  tar xvf libssl_1.1.tar.xz
  sudo mv openssl-1.1/* /usr/local/webots/lib/webots/
fi

# The following packages are only available in the ROS 2 Foxy/Galactic distributions. Therefore, we cannot include them in the package.xml, but we have to install them manually here.

if [[ "${ROS_DISTRO}" == "foxy" || "${ROS_DISTRO}" == "galactic" ]]; then
    apt install -y ros-${ROS_DISTRO}-turtlebot3-cartographer ros-${ROS_DISTRO}-turtlebot3-navigation2
fi

# Setup Qt plugins for RViz (can be used once RViz does not randomly crash anymore in GitHub CI).
#export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
