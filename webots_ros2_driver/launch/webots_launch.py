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

"""Launch Webots Universal Robot simulation."""

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from webots_ros2_driver.webots_launcher import WebotsLauncher


'''

xacro ur.urdf.xacro > ur5e.urdf name:=ur5e joint_limit_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/joint_limits.yaml kinematics_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/default_kinematics.yaml physical_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/physical_parameters.yaml visual_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/visual_parameters.yaml

'''

def generate_launch_description():
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=world,
    )

    '''
    supervisor_spawner = Node(
        package='webots_ros2_driver',
        executable='supervisor_spawner.py',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'Spawner'},
        respawn=True,
    )
    '''

    return LaunchDescription([DeclareLaunchArgument(
            'world',
            description=f'Give a world file path as argument'
        ),
        webots,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
