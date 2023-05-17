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
LAST_COMMIT=$(git log -1 --pretty=format:"%H")
echo "BRANCH_NAME=sync-${BRANCH}-${LAST_COMMIT}" >> $GITHUB_ENV

# Copy headers and source code
cd ${ROS2_HOME}
rm -rf include
mkdir -p include/controller
mkdir -p include/plugins
cp -r ${WEBOTS_HOME}/include/controller/* include/controller
cp -r ${WEBOTS_HOME}/include/plugins/* include/plugins
cp ${WEBOTS_HOME}/include/controller/c/webots/plugins/robot_window/{robot_window.h,robot_wwi.h} include

rm -rf lib
mkdir -p lib/controller/python
cp ${WEBOTS_HOME}/lib/controller/.gitignore lib/controller
cp -r ${WEBOTS_HOME}/lib/controller/python/* lib/controller/python

rm -rf projects
mkdir -p projects/default/libraries/vehicle/c
mkdir -p projects/default/libraries/vehicle/cpp
mkdir -p projects/default/libraries/vehicle/java
cp -r ${WEBOTS_HOME}/projects/default/libraries/vehicle/c/* projects/default/libraries/vehicle/c
cp -r ${WEBOTS_HOME}/projects/default/libraries/vehicle/cpp/* projects/default/libraries/vehicle/cpp
cp -r ${WEBOTS_HOME}/projects/default/libraries/vehicle/java/* projects/default/libraries/vehicle/java
cp ${WEBOTS_HOME}/projects/default/libraries/vehicle/java/.gitignore projects/default/libraries/vehicle/java
cp ${WEBOTS_HOME}/projects/default/libraries/vehicle/Makefile projects/default/libraries/vehicle

rm -rf resources
mkdir -p resources/projects/libraries/generic_robot_window
cp -r ${WEBOTS_HOME}/resources/projects/libraries/generic_robot_window/* resources/projects/libraries/generic_robot_window
cp ${WEBOTS_HOME}/resources/Makefile.include resources
cp ${WEBOTS_HOME}/resources/Makefile.os.include resources
cp ${WEBOTS_HOME}/resources/version.txt resources

rm -rf src
mkdir -p src/controller
mkdir -p src/stb
cp -r ${WEBOTS_HOME}/src/controller/* src/controller
cp -r ${WEBOTS_HOME}/src/stb/* src/stb

rm -f .gitignore
cp ${WEBOTS_HOME}/.gitignore .gitignore
