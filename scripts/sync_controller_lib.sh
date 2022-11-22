#!/bin/bash -

BRANCH=$1

# ROS2_HOME has to be specified
if [ -z "${ROS2_HOME}" ]; then
    ROS2_HOME=$(pwd)/webots_ros2_driver/webots
fi

# Get the webots repo
rm -rf /tmp/webots || true
if [ ! -z "${GITHUB_ACTOR}" ]; then
    git config --global user.name ${GITHUB_ACTOR}
    git config --global user.email ${GITHUB_ACTOR}@github.com
fi
git clone --recurse-submodules -b ${BRANCH} https://github.com/cyberbotics/webots.git /tmp/webots
if [ ! -d /tmp/webots ]; then
    echo 'The repository is not properly cloned'
    exit 1
fi
cd /tmp/webots
WEBOTS_HOME=/tmp/webots

# Don't publish the libcontroller if it hasn't changed since yesterday
LAST_COMMIT_YESTERDAY=$(git log -1 --pretty=format:"%H" --before=yesterday)
LAST_COMMIT=$(git log -1 --pretty=format:"%H")
echo "BRANCH_NAME=sync-${BRANCH}-${LAST_COMMIT}" >> $GITHUB_ENV
INCLUDE_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- include/controller)
SOURCE_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- src/controller)
VEHICLE_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- projects/default/librairies/vehicle)
WINDOW_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- resources/projects/libraries/generic_robot_window)
if [ -z "${INCLUDE_DIFF_SINCE_YESTERDAY}" ] && [ -z "${SOURCE_DIFF_SINCE_YESTERDAY}" ] && [ -z "${VEHICLE_DIFF_SINCE_YESTERDAY}" ] && [ -z "${WINDOW_DIFF_SINCE_YESTERDAY}" ]; then
    echo "There are no changes in 'include/controller', 'src/controller', 'projects/default/librairies/vehicle' and 'resources/projects/libraries/generic_robot_window' since yesterday"
    echo "Last commit yesterday: ${LAST_COMMIT_YESTERDAY}"
    echo "Last commit today: ${LAST_COMMIT}"
    exit 0
fi

# Copy headers and source code
cd ${ROS2_HOME}
rm -rf include
mkdir -p include
cp -r ${WEBOTS_HOME}/include/controller/* include
cp -r ${WEBOTS_HOME}/include/plugins/* include
cp ${WEBOTS_HOME}/include/controller/c/webots/plugins/robot_window/{robot_window.h,robot_wwi.h} include

rm -rf lib
mkdir -p lib/controller
cp ${WEBOTS_HOME}/lib/controller/.gitignore lib/controller
cp -r ${WEBOTS_HOME}/lib/controller/python/* lib/controller

rm -rf projects
mkdir -p projects/default/libraries/vehicle
cp -r ${WEBOTS_HOME}/projects/default/libraries/vehicle/c/* projects/default/libraries/vehicle
cp -r ${WEBOTS_HOME}/projects/default/libraries/vehicle/cpp/* projects/default/libraries/vehicle
cp -r ${WEBOTS_HOME}/projects/default/libraries/vehicle/java/* projects/default/libraries/vehicle
cp ${WEBOTS_HOME}/projects/default/libraries/vehicle/Makefile projects/default/libraries/vehicle

rm -rf resources
mkdir -p resources/projects/libraries
cp -r ${WEBOTS_HOME}/resources/projects/libraries/generic_robot_window/* resources/projects/libraries
cp ${WEBOTS_HOME}/resources/Makefile.include resources
cp ${WEBOTS_HOME}/resources/Makefile.os.include resources

rm -rf src
mkdir -p src
cp -r ${WEBOTS_HOME}/src/controller/* include
cp -r ${WEBOTS_HOME}/src/stb/* include
