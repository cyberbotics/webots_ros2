ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG DOCKERUSER=
ARG DOCKERUSERCOMMENT=
ARG ROS_DISTRO=jazzy
ARG ROS_TESTING=0
ARG WEBOTS_VERSION=2023b

RUN userdel -r ubuntu && \
    useradd -d /${DOCKERUSER} -m \
            -u ${UID} -U \
            -s /usr/bin/bash \
            -c "${DOCKERUSERCOMMENT}" ${DOCKERUSER} && \
    echo "${DOCKERUSER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod -a -G video ${DOCKERUSER} && \
    usermod -a -G dialout ${DOCKERUSER}

RUN apt update && apt install -y wget && \
    wget https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb -O /tmp/webots.deb && \
    apt install -y /tmp/webots.deb && \
    rm /tmp/webots.deb

RUN [ "$ROS_TESTING" -eq "1" ] && echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list || true

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    gdb

COPY ./docker/bashrc /tmp/bashrc
COPY ./docker/webots.conf /${DOCKERUSER}/.config/Cyberbotics/Webots-R${WEBOTS_VERSION}.conf
RUN cat /tmp/bashrc >> /${DOCKERUSER}/.bashrc

ENV USERNAME=default
USER ${DOCKERUSER}
WORKDIR /${DOCKERUSER}/ros2_ws
