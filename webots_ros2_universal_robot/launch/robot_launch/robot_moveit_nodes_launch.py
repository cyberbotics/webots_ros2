#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots Universal Robot simulation nodes with MoveIt2."""

import os
import pathlib
import yaml
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes


PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory(PACKAGE_NAME)

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'resource', filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_file(filename))

    # Check if moveit is installed
    if 'moveit' in get_packages_with_prefixes():
        # Webots simulation with robot
        launch_description_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'robot_nodes_launch.py'))
            )
        )

        # Configuration
        description = {'robot_description': load_file('ur5e_with_gripper.urdf')}
        description_semantic = {'robot_description_semantic': load_file('moveit_ur5e.srdf')}
        description_kinematics = {'robot_description_kinematics': load_yaml('moveit_kinematics.yaml')}
        sim_time = {'use_sim_time': True}

        # Rviz node
        rviz_config_file = os.path.join(package_dir, 'resource', 'moveit_visualization.rviz')

        launch_description_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    sim_time
                ],
            )
        )

        # MoveIt2 node
        movegroup = {'move_group': load_yaml('moveit_movegroup.yaml')}
        moveit_controllers = {
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': load_yaml('moveit_controllers.yaml')
        }

        launch_description_nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    moveit_controllers,
                    movegroup,
                    sim_time
                ],
                remappings=[('/joint_states', '/ur5e/joint_states')]
            )
        )
    else:
        launch_description_nodes.append(LogInfo(msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'))

    return LaunchDescription(launch_description_nodes)
