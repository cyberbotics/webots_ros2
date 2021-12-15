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

import os
import pathlib
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


PACKAGE_NAME = 'webots_ros2_universal_robot'

def get_ros2_control_spawners(event):
    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')
    robot_description = pathlib.Path(urdf_path).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster'] + controller_manager_timeout,
    )

    universal_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5e'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            ros2_control_params
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    if "success=True" in event.text.decode().strip():
        return [
            joint_state_broadcaster_spawner,
            trajectory_controller_spawner,
            robot_state_publisher,
            universal_robot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=universal_robot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    return

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')

    service_send_urdf_robot = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/spawn_urdf_robot',
            'webots_ros2_msgs/srv/SetWbURDFRobot',
            '{\
            "robot": { "name": "UR5e",\
                "urdf_location": "'+urdf_path+'",\
                "translation": "0 0 0.6",\
                "rotation": "0 0 1 -1.5708"\
                }\
            }'
        ]
    )

    publish_clean_urdf_robot = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            '/clean_urdf_robot',
            'std_msgs/msg/Bool',
            '{\
            "data": "True"\
            }'
        ]
    )

    return LaunchDescription([
        service_send_urdf_robot,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=service_send_urdf_robot,
                on_stdout=lambda event: get_ros2_control_spawners(event),
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=service_send_urdf_robot,
                on_exit=publish_clean_urdf_robot,
            )
        ),
    ])


'''
launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=universal_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
'''