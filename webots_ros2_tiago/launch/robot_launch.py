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
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tiago')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_slam_cartographer = LaunchConfiguration('slam_cartographer', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

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

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'tiago_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    tiago_driver = WebotsController(
        robot_name='Tiago_Lite',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
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

    # Navigation
    navigation_nodes = []
    nav2_params_file = 'nav2_params.yaml'
    nav2_params = os.path.join(package_dir, 'resource', nav2_params_file)
    nav2_map = os.path.join(package_dir, 'resource', 'map.yaml')
    if 'nav2_bringup' in get_packages_with_prefixes():
        navigation_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('params_file', nav2_params),
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_nav)))

    # SLAM
    cartographer_config_dir = os.path.join(package_dir, 'resource')
    cartographer_config_basename = 'cartographer.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_basename],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer)

    grid_executable = 'cartographer_occupancy_grid_node'
    cartographer_grid = Node(
        package='cartographer_ros',
        executable=grid_executable,
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer_grid)

    toolbox_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam_toolbox)
    )
    navigation_nodes.append(slam_toolbox)

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=tiago_driver,
        nodes_to_start=[rviz] + navigation_nodes + ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='default.wbt',
            description='Choose one of the world files from `/webots_ros2_tiago/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        tiago_driver,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
