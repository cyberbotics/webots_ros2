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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    package_webots_ros2_driver_dir = get_package_share_directory('webots_ros2_driver')

    # Webots
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_webots_ros2_driver_dir, 'launch', 'webots_launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([package_dir, 'worlds', 'universal_robot.wbt'])
        }.items()
    )

    return LaunchDescription([
        webots_launch,
    ])
