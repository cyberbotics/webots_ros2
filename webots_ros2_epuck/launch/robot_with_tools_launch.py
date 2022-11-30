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

"""Launch Webots epuck driver and the additional tools."""

import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_epuck')

    use_nav = LaunchConfiguration('nav', default=False)
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_mapper = LaunchConfiguration('mapper', default=False)
    synchronization = LaunchConfiguration('synchronization', default=False)
    world = LaunchConfiguration('world', default='epuck_world.wbt')

    # Webots
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'synchronization': synchronization,
            'use_sim_time': 'true'
        }.items()
    )

    # Base configuration
    tools_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_tools_launch.py')
        ),
        launch_arguments={
            'nav': use_nav,
            'rviz': use_rviz,
            'mapper': use_mapper,
            'use_sim_time': 'true',
            'world': world
        }.items()
    )

    return LaunchDescription([
        webots_launch,
        tools_launch
    ])
