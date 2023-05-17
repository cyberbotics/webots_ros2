#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('webots_ros2_tiago')
    use_rviz = LaunchConfiguration('rviz', default=False)
    robot_description_path = os.path.join(package_dir, 'resource', 'tiago_bringup_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_bringup.yml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    tiago_driver = WebotsController(
        robot_name='Tiago',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time},
            ros2_control_params
        ]
    )

    tiago_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_bringup'), 'launch', 'tiago_bringup.launch.py')),
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=tiago_driver,
        nodes_to_start=[tiago_bringup_launch]
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'default_bringup.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return [
        tiago_driver,
        waiting_nodes,
        rviz,
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tiago')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # The following lines are important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='default_bringup.wbt',
            description='Choose one of the world files from `/webots_ros2_tiago/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())
