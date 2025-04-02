#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
# Copyright 2023 Husarion
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

"""Launch Webots ROSbot 2R driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('webots_ros2_husarion')
    robot_description_path = os.path.join(package_dir, 'resource', 'rosbot_webots.urdf')
    links_remappings_file_path = os.path.join(package_dir, 'resource', 'rosbot_links_remappings.yaml')

    ekf_config = os.path.join(package_dir, 'resource', 'ekf.yaml')

    ros2_control_params = os.path.join(package_dir, 'resource', 'rosbot_controllers.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['rosbot_base_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diff_drive_controller_spawner, joint_state_broadcaster_spawner]

    rosbot_driver = WebotsController(
        robot_name='rosbot',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
            },
            ros2_control_params,
            {'components_remappings': links_remappings_file_path}
        ],
        remappings=[
            ('rosbot_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('rosbot_base_controller/cmd_vel', '/cmd_vel'),
            ('rosbot/laser', '/scan'),
            ('rosbot/rl_range', '/range/rl'),
            ('rosbot/rr_range', '/range/rr'),
            ('rosbot/fl_range', '/range/fl'),
            ('rosbot/fr_range', '/range/fr')
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

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True},
            {'odom0': '/rosbot_base_controller/odom'}
        ]
    )

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=rosbot_driver,
        nodes_to_start=ros_control_spawners
    )

    return [
        robot_state_publisher,
        rosbot_driver,
        waiting_nodes,
        robot_localization_node
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_husarion')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # The following line is important!
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
            default_value='rosbot.wbt',
            description='Choose one of the world files from `/webots_ros2_husarion/world` directory'
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
