#!/usr/bin/env bash

ROS_DISTRO=$1
ROS_REPO=$2

WEBOTS_VERSION="R${WEBOTS_RELEASE_VERSION}"

if [ "${TEST_WITH_WEBOTS_NIGTHLY}" == "1" ]; then
    # Take the latest nightly build
    YESTERDAY_WEEK_DAY_NUMBER=`date --date="1 day ago" +"%u"`
    LAST_NIGHTLY_DAY_OLD=1
    # There is no nightly build the weekend
    if [ ${YESTERDAY_WEEK_DAY_NUMBER} -gt 5 ]; then
        LAST_NIGHTLY_DAY_OLD="$((${YESTERDAY_WEEK_DAY_NUMBER}-4))"
    fi
    NIGHTLY_DATE=`date --date="${LAST_NIGHTLY_DAY_OLD} day ago" +"%-d_%-m_%Y"`
    WEBOTS_VERSION="nightly_${NIGHTLY_DATE}"
fi

apt update > /dev/null
apt install -y wget dialog apt-utils psmisc lsb-release git > /dev/null
wget https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots_${WEBOTS_RELEASE_VERSION}_amd64.deb -O /tmp/webots.deb > /dev/null
apt install -y /tmp/webots.deb xvfb > /dev/null

# OpenSSL patch for ubuntu 22
if [[ $(lsb_release -rs) == "22.04" && ${WEBOTS_RELEASE_VERSION} == "2022a" ]]; then
  echo applying openssl patch
  wget https://cyberbotics.com/files/repository/dependencies/linux64/release/libssl_1.1.tar.xz -O /tmp/libssl_1.1.tar.xz
  tar xvf /tmp/libssl_1.1.tar.xz -C /tmp
  mv /tmp/openssl-1.1/* /usr/local/webots/lib/webots/
fi

# The following packages are not available in the ROS 2 Rolling distribution. Therefore, we cannot include them in the package.xml, but we have to install them manually here.
if [[ "${ROS_DISTRO}" != "rolling" ]]; then
    apt install -y ros-${ROS_DISTRO}-nav2-bringup

    # Turtlebot3 must be installed from sources to make RViz optional
    mkdir -p /root/turtlebot_ws/src
    git clone -b ${ROS_DISTRO}-devel https://github.com/cyberbotics/turtlebot3.git /root/turtlebot_ws/src
fi

# TODO: Revert once the https://github.com/ros-planning/navigation2/issues/3033 issue is fixed.
# Fast-DDS is not working properly with the Nav2 package on Humble and Iron. Using Cyclone DDS instead.
if [[ "${ROS_DISTRO}" != "rolling" ]]; then
    apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
fi

# Setup Qt plugins for RViz (can be used once RViz does not randomly crash anymore in GitHub CI).
#export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins


# Fixes:
#   MESA: error: ZINK: failed to choose pdev
#   2024-07-27T19:23:20.1063344Z [webots-4] glx: failed to create drisw screen
if [[ "${ROS_DISTRO}" == "rolling" ]]; then
    apt-get install -y libqt5quickcontrols2-5 qtquickcontrols2-5-dev
fi
