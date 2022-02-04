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
apt install -y wget dialog apt-utils psmisc
wget https://github.com/cyberbotics/webots/releases/download/${WEBOTS_NIGHTLY_VERSION}/webots_${WEBOTS_RELEASE_VERSION}_amd64.deb -O /tmp/webots.deb
apt install -y /tmp/webots.deb xvfb

# The following packages are only available in the ROS 2 Foxy distribution. Therefore, we cannot include them in the package.xml, but we have to install them manually here.
if [ "${ROS_DISTRO}" = "foxy" ]; then
    apt install -y ros-foxy-turtlebot3-cartographer ros-foxy-turtlebot3-navigation2
fi

# Setup Qt plugins for RViz (can be used once RViz does not randomly crash anymore in GitHub CI).
#export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
