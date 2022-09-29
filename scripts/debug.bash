#!/usr/bin/env bash

ROS_DISTRO=$1

sudo apt -y remove ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-rclcpp-lifecycle ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-controller-interface
sudo apt -y install ros-${ROS_DISTRO}-rclpy=1.0.8-1 ros-${ROS_DISTRO}-ros2-control=0.11.0-1 ros-${ROS_DISTRO}-ros2-controllers=0.8.2-1 ros-${ROS_DISTRO}-rclcpp=2.4.2-1 ros-${ROS_DISTRO}-rclcpp-lifecycle=2.4.2-1 ros-${ROS_DISTRO}-controller-manager=0.11.0-1 ros-${ROS_DISTRO}-controller-interface=0.11.0-1
