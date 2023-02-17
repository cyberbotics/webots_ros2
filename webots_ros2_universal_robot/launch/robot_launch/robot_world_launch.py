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

"""Launch Webots Universal Robot simulation world."""

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    # Starts Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='universal_robot.wbt',
            description='Choose one of the world files from `/webots_ros2_universal_robot/worlds` directory'
        ),
        webots,
        webots._supervisor,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
