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

"""Launch Webots and the controller."""

import os

from pathlib import Path

import launch
import launch_ros.actions

from webots_ros2_core.utils import ControllerLauncher

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    arguments = ['--mode=realtime', '--world=' +
                 os.path.join(get_package_share_directory('webots_ros2_universal_robot'),
                              'worlds', 'universal_robot_rviz.wbt')]
    webots = launch_ros.actions.Node(package='webots_ros2_core', node_executable='webots_launcher',
                                     arguments=arguments, output='screen')
    # Controller nodes
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    URe5Controller = ControllerLauncher(package='webots_ros2_universal_robot',
                                        node_executable='universal_robot',
                                        # this argument should match the 'name' field
                                        # of the robot in Webots
                                        arguments=['--webots-robot-name=UR5e'],
                                        parameters=[{'synchronization': synchronization}],
                                        output='screen')
    tfController = ControllerLauncher(package='webots_ros2_core',
                                      node_executable='tf_publisher',
                                      # this argument should match the 'name' field
                                      # of the robot in Webots
                                      arguments=['--webots-robot-name=tf_supervisor'],
                                      parameters=[{'synchronization': synchronization}],
                                      output='screen')
    # Copy .rviz config file and update path ro URDF file.
    templateRvizFile = os.path.join(get_package_share_directory('webots_ros2_ur_e_description'),
                                    'rviz', 'view_robot') + '.rviz'
    home = Path.home()
    customRvizFile = os.path.join(home, 'webots_ros2_ur_e_description.rviz')
    if not os.path.exists(os.path.join(home, 'webots_ros2_ur_e_description.rviz')):
        with open(templateRvizFile, 'r') as f:
            content = f.read()
            content = content.replace('package://webots_ros2_ur_e_description',
                                      get_package_share_directory('webots_ros2_ur_e_description'))
            with open(customRvizFile, 'w') as f2:
                f2.write(content)
    # Rviz node
    rviz = launch_ros.actions.Node(package='rviz2',
                                   node_executable='rviz2',
                                   arguments=['-d', customRvizFile],
                                   output='screen')
    return launch.LaunchDescription([
        rviz,
        webots,
        URe5Controller,
        tfController,
        # Shutdown launch when Webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
