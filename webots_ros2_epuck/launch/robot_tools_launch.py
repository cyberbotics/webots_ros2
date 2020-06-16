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

"""Launch additional tools for e-puck controller to allow visualization, mapping and navigation."""

import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory('webots_ros2_epuck')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_mapper = LaunchConfiguration('mapper', default=False)
    fill_map = LaunchConfiguration('fill_map', default=True)

    # Rviz node
    rviz_config = os.path.join(package_dir, 'resource', 'all.rviz')
    launch_description_nodes.append(
        Node(
            package='rviz2',
            node_executable='rviz2',
            output='log',
            arguments=['--display-config=' + rviz_config],
            condition=launch.conditions.IfCondition(use_rviz)
        )
    )

    # Navigation
    if 'nav2_bringup' in get_packages_with_prefixes():
        launch_description_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'params_file': os.path.join(package_dir, 'resource', 'nav2_params.yaml'),
                    'use_sim_time': use_sim_time
                }.items(),
                condition=launch.conditions.IfCondition(use_nav)
            )
        )
    else:
        print('Navigation2 is not installed, navigation functionality is disabled')

    # Mapping
    launch_description_nodes.append(
        Node(
            package='webots_ros2_epuck',
            node_executable='simple_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'fill_map': fill_map}],
            condition=launch.conditions.IfCondition(use_mapper)
        )
    )

    # Launch descriptor
    return LaunchDescription(launch_description_nodes)
