#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots Universal Robot simulation."""

import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'webots_ros2_universal_robot'

'''
xacro ur.urdf.xacro > ur5e.urdf name:=ur5e joint_limit_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/joint_limits.yaml kinematics_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/default_kinematics.yaml physical_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/physical_parameters.yaml visual_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/visual_parameters.yaml 

'''

def generate_launch_description():
    world = LaunchConfiguration('world')

    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')
    robot_description = pathlib.Path(urdf_path).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    # Define your URDF robots here
    # Names of the robots have to match the environment variable convention and have to be unique
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        robots=[
            {'name': 'UR5e',
             'urdf_location': urdf_path,
             'translation': '0 0 0.6',
             'rotation': '0 0 1 -1.5708',
            },
        ]
    )

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster'] + controller_manager_timeout,
    )

    universal_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            ros2_control_params
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Use webots.getDriversList() to get all the drivers node, the basis of the
    # webots_ros2 interface.
    return LaunchDescription([DeclareLaunchArgument(
            'world',
            default_value='universal_robot.wbt',
            description=f'Choose one of the world files from `/{PACKAGE_NAME}/world` directory'
        ),
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        webots,
        robot_state_publisher,
        universal_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
