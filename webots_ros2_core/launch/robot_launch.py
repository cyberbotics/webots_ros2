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

"""Launch Webots and ROS2 driver."""

import os
import sys
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.utils import ControllerLauncher


def generate_launch_description():
    synchronization = LaunchConfiguration('synchronization', default=False)

    params = {arg.split(':=')[0]: arg.split(':=')[1] for arg in sys.argv if ':=' in arg}

    # Webots
    webots = Node(
        package='webots_ros2_core',
        node_executable='webots_launcher',
        arguments=[
            '--mode=realtime',
            '--world=' + params['world']
        ],
        output='screen'
    )

    # Driver node
    controller = ControllerLauncher(
        package='webots_ros2_core',
        node_executable='webots_differential_drive_node',
        parameters=[{
            'synchronization': synchronization,
            'wheel_distance': 0.404,
            'wheel_radius': 0.1955
        }],
        output='screen'
    )

    return LaunchDescription([
        webots,
        controller,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
