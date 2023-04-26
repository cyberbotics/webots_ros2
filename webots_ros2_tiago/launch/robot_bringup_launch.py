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
import pathlib
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
from webots_ros2_driver.utils import controller_url_prefix


def launch_spawners(event, nodes):
    # Start ros2_control spawners once the controller_manager is ready
    if 'Successful \'activate\' of hardware' in event.text.decode().strip():
        return nodes
    return


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('webots_ros2_tiago')
    use_rviz = LaunchConfiguration('rviz', default=False)
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_bringup_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_bringup.yml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    tiago_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Tiago'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ]
    )

    # Tiago bringup specific nodes
    play_motion2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_bringup'), 'launch', 'tiago_play_motion2.launch.py')),
    )

    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_bringup'), 'launch', 'twist_mux.launch.py')),
    )

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_bringup'), 'launch', 'joystick_teleop.launch.py')),
    )

    # ROS2_control (bringup controllers with custom parametersf for Webots)
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    pal_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['gripper_controller'] + controller_manager_timeout,
    )

    mobile_base_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_controller_configuration'), 'launch', 'mobile_base_controller.launch.py')),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    torso_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_controller_configuration'), 'launch', 'torso_controller.launch.py')),
    )

    head_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_controller_configuration'), 'launch', 'head_controller.launch.py')),
    )

    arm_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tiago_controller_configuration'), 'launch', 'arm_controller.launch.py')),
    )

    spawners = []
    spawners.append(pal_gripper_controller_spawner)
    spawners.append(mobile_base_controller_launch)
    spawners.append(joint_state_broadcaster_spawner)
    spawners.append(torso_controller_launch)
    spawners.append(head_controller_launch)
    spawners.append(arm_controller_launch)

    spawners_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessIO(
            target_action=tiago_driver,
            on_stderr=lambda event: launch_spawners(event, spawners)
        )
    )

    # State publishers
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return [
        footprint_publisher,
        robot_state_publisher,
        tiago_driver,
        play_motion2,
        twist_mux,
        joystick_teleop,
        spawners_handler,
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
