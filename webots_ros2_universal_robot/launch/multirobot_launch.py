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

"""Launch Webots Universal Robot and ABB Robot simulation."""

import os
import xacro
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


PACKAGE_NAME = 'webots_ros2_universal_robot'


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    package_dir = get_package_share_directory(PACKAGE_NAME)
    ur5e_xacro_path = os.path.join(package_dir, 'resource', 'ur5e_with_gripper.urdf.xacro')
    ur5e_description = xacro.process_file(ur5e_xacro_path, mappings={'name': 'UR5eWithGripper'}).toxml()
    abb_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_abb_description.urdf')).read_text()
    ur5e_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')
    abb_control_params = os.path.join(package_dir, 'resource', 'ros2_control_abb_config.yaml')

    # Define your URDF robots here
    # The name of an URDF robot has to match the WEBOTS_CONTROLLER_URL of the driver node
    # You can specify the URDF content to use with robot_description
    # In case you have relative paths in your URDF, specifiy the relative_path_prefix as the directory of your xacro file
    spawn_URDF_ur5e = URDFSpawner(
        name='UR5e',
        robot_description=ur5e_description,
        relative_path_prefix=os.path.join(package_dir, 'resource'),
        translation='0 0 0.62',
        rotation='0 0 1 -1.5708',
    )

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_CONTROLLER_URL` has to match the robot name in the world file.
    # You can check for more information at:
    # https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers
    ur5e_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'UR5e'},
        namespace='ur5e',
        parameters=[
            {'robot_description': ur5e_description},
            {'use_sim_time': True},
            ur5e_control_params
        ]
    )

    # Standard Webots robot using driver node
    abb_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'abbirb4600'},
        namespace='abb',
        parameters=[
            {'robot_description': abb_description},
            {'use_sim_time': True},
            abb_control_params
        ]
    )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '75']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'
    ur5e_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    ur5e_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    abb_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['abb_joint_trajectory_controller', '-c', 'abb/controller_manager'] + controller_manager_timeout,
    )
    abb_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
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

    return [
        # Request to spawn the URDF robot
        spawn_URDF_ur5e,

        # Standard Webots robot
        abb_driver,

        # Other ROS 2 nodes
        ur5e_trajectory_controller_spawner,
        ur5e_joint_state_broadcaster_spawner,
        abb_trajectory_controller_spawner,
        abb_joint_state_broadcaster_spawner,

        # Launch the driver node once the URDF robot is spawned.
        # You might include other nodes to start them with the driver node.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF_ur5e,
                on_stdout=lambda event: get_webots_driver_node(event, [ur5e_driver, ur5e_controller, abb_controller]),
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    # Starts Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
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
            default_value='robotic_arms.wbt',
            description='Choose one of the world files from `/webots_ros2_universal_robot/worlds` directory'
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
