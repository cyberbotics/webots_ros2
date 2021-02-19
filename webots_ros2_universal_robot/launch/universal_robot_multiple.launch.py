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

"""Launch Webots and the controller."""

import os
import launch
from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(
        world=os.path.join(
            get_package_share_directory('webots_ros2_universal_robot'),
            'worlds',
            'universal_robot_multiple.wbt'
        )
    )

    # Controller nodes
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    ure3_controller = ControllerLauncher(
        package='webots_ros2_core',
        executable='webots_robotic_arm_node',
        arguments=['--webots-robot-name=UR3e'],
        namespace='UR3e',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )
    ure5_controller = ControllerLauncher(
        package='webots_ros2_core',
        executable='webots_robotic_arm_node',
        arguments=['--webots-robot-name=UR5e'],
        namespace='UR5e',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )
    return launch.LaunchDescription([
        webots, ure3_controller, ure5_controller,
        # Shutdown launch when Webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
