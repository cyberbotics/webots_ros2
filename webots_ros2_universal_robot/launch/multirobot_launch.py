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

"""Launch Webots and the controllers."""

import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    ur5e_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_ur5e_description.urdf')).read_text()
    abb_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_abb_description.urdf')).read_text()
    ur5e_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')
    abb_control_params = os.path.join(package_dir, 'resource', 'ros2_control_abb_config.yaml')

    # Webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'armed_robots.wbt'))

    # Driver nodes
    ur5e_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5e'},
        parameters=[
            {'robot_description': ur5e_description},
            ur5e_control_params
        ]
    )
    abb_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'abbirb4600'},
        parameters=[
            {'robot_description': abb_description},
            abb_control_params
        ]
    )

    # Control nodes
    """
    ur5e_controller = Node(
        package='webots_ros2_driver',
        executable='armed_robots_ur',
        output='screen'
    )
    abb_controller = Node(
        package='webots_ros2_driver',
        executable='armed_robots_abb',
        output='screen'
    )
    """

    return launch.LaunchDescription([
        webots,
        # abb_controller,
        # ur5e_controller,
        ur5e_driver,
        abb_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
