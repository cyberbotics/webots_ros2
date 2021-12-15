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


def generate_launch_description():
    world = LaunchConfiguration('world')

    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')

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

    supervisor_spawner = Node(
        package='webots_ros2_universal_robot',
        executable='supervisor_spawner',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'supervisor'},
        respawn=True,
    )

    return LaunchDescription([DeclareLaunchArgument(
            'world',
            default_value='universal_robot.wbt',
            description=f'Choose one of the world files from `/{PACKAGE_NAME}/world` directory'
        ),
        webots,
        supervisor_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
