#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch MoveIt2 in RViz2."""

# Reference: https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes/run_move_group/launch/run_move_group.launch.py

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_universal_robot')

    # Webots
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'executable': 'webots_robotic_arm_node',
            'world': os.path.join(package_dir, 'worlds', 'universal_robot_rviz.wbt')
        }.items()
    )

    # MoveIt Config
    robot_description_config = None
    with open('/tmp/webots_robot_base_link.urdf', 'r') as file:
        robot_description_config = file.read()
    robot_description = {'robot_description': robot_description_config}
    robot_description_semantic = {
        'robot_description_semantic': load_file('webots_ros2_universal_robot', 'config/ur5e.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('webots_ros2_universal_robot', 'config/kinematics.yaml')
    }
    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml('webots_ros2_universal_robot', 'config/controllers.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_semantic,
            robot_description,
            moveit_controllers,
            robot_description_kinematics
        ]
    )

    # RViz
    rviz_config_file = os.path.join(package_dir, 'launch', 'universal_robot_moveit2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic
        ]
    )

    return LaunchDescription([webots, rviz_node, run_move_group_node])
