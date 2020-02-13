#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

import launch
import launch_ros.actions

from webots_ros2_core.utils import ControllerLauncher

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    arguments = ['--mode=realtime', '--world=' +
                 os.path.join(get_package_share_directory('webots_ros2_demos'),
                              'worlds', 'armed_robots.wbt')]
    webots = launch_ros.actions.Node(package='webots_ros2_core', node_executable='webots_launcher',
                                     arguments=arguments, output='screen')
    # Controller nodes
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    AbbController = ControllerLauncher(package='webots_ros2_abb',
                                       node_executable='abb_driver',
                                       # this argument should match the 'name' field
                                       # of the robot in Webots
                                       arguments=['--webots-robot-name=abbirb4600'],
                                       node_namespace='abb',
                                       parameters=[{'synchronization': synchronization}],
                                       output='screen')
    Ure5controller = ControllerLauncher(package='webots_ros2_universal_robot',
                                        node_executable='universal_robot',
                                        # this argument should match the 'name' field
                                        # of the robot in Webots
                                        arguments=['--webots-robot-name=UR5e'],
                                        node_namespace='ur',
                                        parameters=[{'synchronization': synchronization}],
                                        output='screen')
    # Control nodes
    armedRobotsUr = ControllerLauncher(package='webots_ros2_demos',
                                       node_executable='armed_robots_ur',
                                       output='screen')
    armedRobotsAbb = ControllerLauncher(package='webots_ros2_demos',
                                        node_executable='armed_robots_abb',
                                        output='screen')
    return launch.LaunchDescription([
        webots, AbbController, Ure5controller, armedRobotsUr, armedRobotsAbb,
        # Shutdown launch when Webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
