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

"""Launch Rat's Life world with navigation."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch_ros.actions import Node


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory('webots_ros2_epuck')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world', default='rats_life_benchmark.wbt')

    # Webots node
    launch_description_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'robot_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'world': world
            }.items()
        )
    )

    # Check if nav2_bringup is installed
    if 'nav2_bringup' in get_packages_with_prefixes():
        # Rviz node
        launch_description_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        )

        # Navigation
        nav2_map = os.path.join(package_dir, 'resource', 'map_rats_life.yaml')

        launch_description_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                ),
                launch_arguments=[
                    ('map', nav2_map),
                    ('use_sim_time', use_sim_time),
                    ('params_file', os.path.join(package_dir, 'resource', 'nav2_params.yaml'))
                ],
            )
        )
    else:
        launch_description_nodes.append(LogInfo(msg='Navigation2 is not installed, navigation functionality is disabled'))

    # Launch descriptor
    return LaunchDescription(launch_description_nodes)
