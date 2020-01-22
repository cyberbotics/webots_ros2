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

from ament_index_python.packages import get_package_share_directory

from webots_ros2_core.utils import ControllerLauncher


def generate_launch_description():
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
    # Webots
    arguments = ['--mode=realtime', '--world=' +
                 os.path.join(get_package_share_directory('webots_ros2_demos'),
                              'worlds', 'armed_robots.wbt')]
    webots = launch_ros.actions.Node(package='webots_ros2_core', node_executable='webots_launcher',
                                     arguments=arguments, output='screen')
    # Gazebo
    webots_ros2_aws = get_package_share_directory('webots_ros2_aws')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(webots_ros2_aws, 'worlds', 'empty.world'), ''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        AbbController, Ure5controller, webots, gazebo_server, gazebo_client,
        # Shutdown launch when Webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
