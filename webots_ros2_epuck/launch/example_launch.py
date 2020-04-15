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
#
# ros2 launch webots_ros2_epuck example_launch.py

"""Launch Webots, the controller and Rviz."""

import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent, IncludeLaunchDescription
from launch_ros.actions import Node
from webots_ros2_core.utils import ControllerLauncher
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_epuck')

    use_nav = LaunchConfiguration('nav', default=False)
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_mapper = LaunchConfiguration('mapper', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    synchronization = LaunchConfiguration('synchronization', default=False)

    # Webots
    arguments = [
        '--mode=realtime',
        '--world=' + os.path.join(package_dir, 'worlds', 'epuck_world.wbt')
    ]
    webots = Node(package='webots_ros2_core', node_executable='webots_launcher',
                  arguments=arguments, output='screen')

    # Driver node
    controller = ControllerLauncher(
        package='webots_ros2_epuck',
        node_executable='driver',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )

    # Base configuration
    base_configuration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'example_tools_launch.py')
        ),
        launch_arguments={
            'nav': use_nav,
            'rviz': use_rviz,
            'mapper': use_mapper,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        webots,
        controller,
        base_configuration,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
