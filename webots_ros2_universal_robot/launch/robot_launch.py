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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription, action
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.actions import ExecuteProcess


from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)

import sys

from std_msgs.msg import String, Bool


PACKAGE_NAME = 'webots_ros2_universal_robot'

'''
xacro ur.urdf.xacro > ur5e.urdf name:=ur5e joint_limit_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/joint_limits.yaml kinematics_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/default_kinematics.yaml physical_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/physical_parameters.yaml visual_params:=/home/benjamin/ros2_ws/src/webots_ros2/webots_ros2_universal_robot/resource/ur_description/config/ur5e/visual_parameters.yaml

'''

def generate_launch_description():
    world = LaunchConfiguration('world')

    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')

    # Define your URDF robots here
    # Names of the robots have to match the environment variable convention and have to be unique
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        robots=[
            {'name': 'UR5e',
             'urdf_location': urdf_path,
             'translation': '0 0 0.6',
             'rotation': '0 0 1 -1.5708',
            },
        ]
    )
    '''
    robots=[
            {'name': 'UR5e',
             'urdf_location': urdf_path,
             'translation': '0 0 0.6',
             'rotation': '0 0 1 -1.5708',
            },
        ]
    '''

    supervisor_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'supervisor'},
        parameters=[
            {'robot_description': pathlib.Path(os.path.join(package_dir, 'resource', 'supervisor_webots.urdf')).read_text()},
        ],
        respawn=True,
    )

    send_urdf_robots = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            '/urdf_robot',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{\
            "header": { "frame_id": "map" },\
            "pose": { "pose": {\
                "position": { "x": 0.005, "y": 0.0, "z": 0.0 },\
                "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }}\
            }\
        }'
        ]
    )

    test_listen_topic = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'echo',
            '/test',
        ]
    )

    # Use webots.getDriversList() to get all the drivers node, the basis of the
    # webots_ros2 interface.
    return LaunchDescription([DeclareLaunchArgument(
            'world',
            default_value='universal_robot.wbt',
            description=f'Choose one of the world files from `/{PACKAGE_NAME}/world` directory'
        ),
        #supervisor_node,
        webots,
        supervisor_driver,
        test_listen_topic,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=supervisor_driver,
                on_stdout=lambda event: test(event),

            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=test_listen_topic,
                on_stdout=lambda event: test2(),

            )
        ),

    ])

def test2():
    return LogInfo(msg='! hello clock !')

def test(event):
    if "had 2 childr" not in event.text.decode().strip():
        return LogInfo(msg=' ###!!!###!!!### BAD Spawn request says "{}"'.format(
                            event.text.decode().strip()))
    return LogInfo(msg=' ###!!!###!!!### GOOD Spawn request says "{}"'.format(
                            event.text.decode().strip()))
