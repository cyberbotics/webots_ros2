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

"""Launch Webots and ROS2 driver."""

import os
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher


ARGUMENTS = [
    DeclareLaunchArgument(
        'world',
        description='Path to Webots\' world file. Make sure `controller` field is set to `<extern>`.'
    ),
    DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Whether to start GUI or not.'
    ),
    DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Choose the startup mode (it must be either `pause`, `realtime`, `run` or `fast`).'
    )
]


def generate_launch_description():
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    mode = LaunchConfiguration('mode')

    # Webots
    webots = WebotsLauncher(
        world=world,
        mode=mode,
        gui=gui
    )

    return LaunchDescription(ARGUMENTS + [
        webots,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
