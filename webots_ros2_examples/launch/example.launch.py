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
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    if 'WEBOTS_HOME' not in os.environ:
        sys.exit('"WEBOTS_HOME" should be defined.')

    arguments = ['--mode=realtime', '--world=' +
                 os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'worlds', 'ros_example.wbt')]
    webots = launch_ros.actions.Node(package='webots_ros2_core', node_executable='webots_launcher',
                                     arguments=arguments, output='screen')
    controller = launch_ros.actions.Node(package='webots_ros2_examples',
                                         node_executable='example_controller',
                                         output='screen')
    os.environ['LD_LIBRARY_PATH'] = (os.environ['WEBOTS_HOME'] + os.sep + 'lib:' +
                                     os.environ.get('LD_LIBRARY_PATH'))
    # os.environ['WEBOTS_PID'] = TODO
    return launch.LaunchDescription([
        webots,
        controller,
        # Shutdown launch when webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
