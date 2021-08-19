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
import yaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'webots_ros2_universal_robot'


def get_moveit_nodes(condition):
    package_dir = get_package_share_directory(PACKAGE_NAME)

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'resource', filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_file(filename))

    description = {'robot_description': load_file('moveit_ur5e_description.urdf')}
    description_semantic = {'robot_description_semantic': load_file('moveit_ur5e.srdf')}
    description_kinematics = {'robot_description_kinematics': load_yaml('moveit_kinematics.yaml')}
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': load_yaml('moveit_controllers.yaml')
    }
    sim_time = {'use_sim_time': True}
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            description,
            description_semantic,
            description_kinematics,
            moveit_controllers,
            sim_time
        ],
        condition=launch.conditions.IfCondition(condition)
    )

    rviz_config_file = os.path.join(package_dir, 'launch', 'universal_robot_moveit2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            description,
            description_semantic,
            description_kinematics,
            sim_time
        ],
        condition=launch.conditions.IfCondition(condition)
    )

    return [run_move_group_node, rviz_node]


def generate_launch_description():
    world = LaunchConfiguration('world')
    use_moveit = LaunchConfiguration('use_moveit')

    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_ur5e_description.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )

    # TODO: Revert once the https://github.com/ros-controls/ros2_control/pull/444 PR gets into the release
    sleep_time = 30 if 'CI' in os.environ else 5
    controller_manager_timeout = ['--controller-manager-timeout', '50'] if os.name == 'nt' else []
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else f"bash -c 'sleep {sleep_time}; $0 $@' "

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
        parameters=[
            {'robot_description': robot_description},
            ros2_control_params
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='universal_robot.wbt',
            description=f'Choose one of the world files from `/{PACKAGE_NAME}/world` directory'
        ),
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        webots,
        robot_state_publisher,
        universal_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),

        # Optional
        DeclareLaunchArgument(
            'use_moveit',
            default_value='false',
        ),

    ] + get_moveit_nodes(use_moveit))
