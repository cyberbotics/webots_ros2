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

# RUN mkdir -p /${DOCKERUSER}/ros2_libs_ws/src && \
#     curl https://raw.githubusercontent.com/ros-controls/ros2_control/refs/heads/master/ros2_control.rolling.repos > /${DOCKERUSER}/ros2_libs_ws/src.repos && \
#     vcs import /${DOCKERUSER}/ros2_libs_ws/src < /${DOCKERUSER}/ros2_libs_ws/src.repos && \
#     git clone https://github.com/ros-controls/ros2_controllers.git /${DOCKERUSER}/ros2_libs_ws/src/ros2_controllers && \
#     git clone --branch=remove/robot_description_param https://github.com/pal-robotics-forks/ros2_control.git /${DOCKERUSER}/ros2_libs_ws/src/ros2_control && \
#     . /opt/ros/${ROS_DISTRO}/setup.sh && \
#     rosdep install --from-paths /${DOCKERUSER}/ros2_libs_ws/src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
#     cd /${DOCKERUSER}/ros2_libs_ws && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./docker/bashrc /tmp/bashrc
COPY ./docker/webots.conf /${DOCKERUSER}/.config/Cyberbotics/Webots-R${WEBOTS_VERSION}.conf
RUN cat /tmp/bashrc >> /${DOCKERUSER}/.bashrc

ENV USERNAME=default
USER ${DOCKERUSER}
WORKDIR /${DOCKERUSER}/ros2_ws
