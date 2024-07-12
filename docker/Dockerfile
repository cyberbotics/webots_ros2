ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG DOCKERUSER=
ARG DOCKERUSERCOMMENT=
ARG ROS_DISTRO=jazzy

RUN userdel -r ubuntu && \
    useradd -d /${DOCKERUSER} -m \
            -u ${UID} -U \
            -s /usr/bin/bash \
            -c "${DOCKERUSERCOMMENT}" ${DOCKERUSER} && \
    echo "${DOCKERUSER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod -a -G video ${DOCKERUSER} && \
    usermod -a -G dialout ${DOCKERUSER}

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers

COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /${DOCKERUSER}/.bashrc

ENV USERNAME default
USER ${DOCKERUSER}
WORKDIR /${DOCKERUSER}/ros2_ws
