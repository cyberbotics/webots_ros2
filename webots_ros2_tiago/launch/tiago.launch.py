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

"""Launch Webots and the controller."""

import os

import launch
import launch_ros.actions

from webots_ros2_core.utils import ControllerLauncher

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    arguments = ['--mode=realtime', '--world=' +
                 os.path.join(get_package_share_directory('webots_ros2_tiago'),
                              'worlds', 'ros_tiago.wbt')]
    webots = launch_ros.actions.Node(package='webots_ros2_core', node_executable='webots_launcher',
                                     arguments=arguments, output='screen')
    # Controller node
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    controller = ControllerLauncher(
        package='webots_ros2_core',
        node_executable='webots_differential_drive_node',
        parameters=[{
            'synchronization': synchronization,
            'wheel_distance': 0.404,
            'wheel_radius': 0.1955,
            'left_joint': 'wheel_left_joint',
            'right_joint': 'wheel_right_joint',
            'left_encoder': 'wheel_left_joint_sensor',
            'right_encoder': 'wheel_right_joint_sensor'
        }],
        arguments=['--name=tiago_driver'],
        output='screen'
    )

    # Rviz node
    use_rviz = launch.substitutions.LaunchConfiguration('rviz', default=False)
    rviz_config = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'odometry.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        node_executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([
        webots,
        controller,
        rviz,
        # Shutdown launch when Webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
