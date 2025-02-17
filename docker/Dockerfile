ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG DOCKERUSER=
ARG DOCKERUSERCOMMENT=
ARG ROS_DISTRO=jazzy
ARG ROS_TESTING=0
ARG WEBOTS_VERSION=2025a

RUN if [ "${ROS_DISTRO}" != "humble" ]; then userdel -r ubuntu; fi
RUN useradd -d /${DOCKERUSER} -m \
            -u ${UID} -U \
            -s /usr/bin/bash \
            -c "${DOCKERUSERCOMMENT}" ${DOCKERUSER} && \
    echo "${DOCKERUSER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod -a -G video ${DOCKERUSER} && \
    usermod -a -G dialout ${DOCKERUSER}

RUN echo $WEBOTS_VERSION | grep -q "nightly" && export WEBOTS_URL="https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}_amd64.deb" || export WEBOTS_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb" && \
    apt update && apt install -y wget && \
    wget ${WEBOTS_URL} -O /tmp/webots.deb && \
    apt install -y /tmp/webots.deb && \
    rm /tmp/webots.deb

RUN [ "$ROS_TESTING" -eq "1" ] && echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list || true

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    gdb \
    git

# HOTFIX: https://github.com/ros-controls/ros2_control/pull/1960
RUN [ "${ROS_DISTRO}" = "humble" ] && wget -O /tmp/hotfix.deb http://snapshots.ros.org/humble/2024-08-28/ubuntu/pool/main/r/ros-humble-hardware-interface/ros-humble-hardware-interface_2.43.0-1jammy.20240823.145349_amd64.deb && \
    apt install -y --allow-downgrades /tmp/hotfix.deb && \
    rm -f /tmp/hotfix.deb || true

COPY ./docker/bashrc /tmp/bashrc
COPY ./docker/webots.conf /${DOCKERUSER}/.config/Cyberbotics/Webots-R${WEBOTS_VERSION}.conf
RUN cat /tmp/bashrc >> /${DOCKERUSER}/.bashrc

ENV USERNAME=default
USER ${DOCKERUSER}
WORKDIR /${DOCKERUSER}/ros2_ws
