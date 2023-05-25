#!/usr/bin/env python

# Copyright 2023 Husarion
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''Launch Webots ROSbot driver.'''

import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import LaunchConfigurationEquals
from launch.actions import OpaqueFunction


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_husarion')
    rosbot_description_package = get_package_share_directory('rosbot_description')
    rosbot_description = os.path.join(rosbot_description_package, 'urdf', 'rosbot.urdf.xacro')
    rosbot_description_urdf = Command(['xacro ', rosbot_description, ' use_sim:=true simulation_engine:=webots'])

    rosbot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': rosbot_description_urdf},
            {'use_sim_time': True},
        ],
    )

    world = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'rosbot.wbt'),
        ros2_supervisor=True
    )

    rosbot_ros2_control_params = os.path.join(
        package_dir, 'resource', 'rosbot_controllers.yaml')

    rosbot_webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'rosbot'},
        output='screen',
        parameters=[
            {'robot_description': rosbot_description_urdf},
            {'set_robot_state_publisher': True},
            {'use_sim_time': True},
            rosbot_ros2_control_params,
        ],
        remappings=[
            ('rosbot_base_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('odom', 'rosbot_base_controller/odom'),
            ('rosbot/laser', '/scan'),
            ('rosbot/rl_range', '/range/rl'),
            ('rosbot/rr_range', '/range/rr'),
            ('rosbot/fl_range', '/range/fl'),
            ('rosbot/fr_range', '/range/fr')
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rosbot_base_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    ekf_config = os.path.join(package_dir, 'resource', 'ekf.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True},
            {'odom0': '/' + 'rosbot_base_controller/odom'}
        ]
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        world,
        world._supervisor,
        rosbot_webots_robot_driver,
        RegisterEventHandler(
                        event_handler=launch.event_handlers.OnProcessExit(
                            target_action=world._supervisor,
                            on_exit=[launch.actions.EmitEvent(
                                event=launch.events.Shutdown())],
                        )
        ),
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        rosbot_state_publisher,
        robot_localization_node
    ])
