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

"""Launch Webots and the controllers."""

import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    ur5e_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_ur5e_description.urdf')).read_text()
    abb_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_abb_description.urdf')).read_text()
    ur5e_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')
    abb_control_params = os.path.join(package_dir, 'resource', 'ros2_control_abb_config.yaml')

    # Webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'armed_robots.wbt'))

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_ROBOT_NAME` has to match the robot name in the world file.
    # You can check for more information at:
    # https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers
    ur5e_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5e'},
        namespace='ur5e',
        parameters=[
            {'robot_description': ur5e_description},
            {'use_sim_time': True},
            ur5e_control_params
        ]
    )
    abb_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'abbirb4600'},
        namespace='abb',
        parameters=[
            {'robot_description': abb_description},
            {'use_sim_time': True},
            abb_control_params
        ]
    )

    controller_manager_timeout = ['--controller-manager-timeout', '75']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    ur5e_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    ur5e_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    abb_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['abb_joint_trajectory_controller', '-c', 'abb/controller_manager'] + controller_manager_timeout,
    )
    abb_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['abb_joint_state_broadcaster', '-c', 'abb/controller_manager'] + controller_manager_timeout,
    )

    # Control nodes
    ur5e_controller = Node(
        package=PACKAGE_NAME,
        executable='ur5e_controller',
        namespace='ur5e',
        output='screen'
    )
    abb_controller = Node(
        package=PACKAGE_NAME,
        executable='abb_controller',
        namespace='abb',
        output='screen'
    )

    return launch.LaunchDescription([
        webots,
        abb_controller,
        ur5e_controller,
        ur5e_driver,
        abb_driver,
        ur5e_trajectory_controller_spawner,
        ur5e_joint_state_broadcaster_spawner,
        abb_trajectory_controller_spawner,
        abb_joint_state_broadcaster_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
