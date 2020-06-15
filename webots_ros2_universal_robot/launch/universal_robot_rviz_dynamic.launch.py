#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os

import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.utils import ControllerLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_universal_robot')
    synchronization = LaunchConfiguration('synchronization', default=False)

    # Webots
    arguments = [
        '--mode=realtime',
        '--world=' + os.path.join(package_dir, 'worlds', 'universal_robot_rviz.wbt')
    ]
    webots = Node(
        package='webots_ros2_core',
        node_executable='webots_launcher',
        arguments=arguments,
        output='screen'
    )

    # Controller nodes
    controller = ControllerLauncher(
        package='webots_ros2_universal_robot',
        node_executable='universal_robot',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )

    # Robot state publisher
    initial_robot_description = '<?xml version="1.0"?><robot name="dummy"><link name="base_link"></link></robot>'
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': initial_robot_description}]
    )

    # Rviz node
    rviz_config = os.path.join(package_dir, 'resource', 'view_robot_dynamic.rviz')
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        output='log',
        arguments=['--display-config=' + rviz_config]
    )

    return LaunchDescription([
        rviz,
        webots,
        controller,
        robot_state_publisher,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
