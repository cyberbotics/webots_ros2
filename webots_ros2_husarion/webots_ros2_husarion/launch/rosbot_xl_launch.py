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


'''Launch Webots rosbot_xl XL driver.'''

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

    rosbot_xl_description_package = get_package_share_directory('rosbot_xl_description')
    rosbot_xl_description = os.path.join(rosbot_xl_description_package, 'urdf', 'rosbot_xl.urdf.xacro')
    rosbot_xl_description_urdf = Command(['xacro ', rosbot_xl_description, ' use_sim:=true simulation_engine:=webots mecanum:=true lidar_model:=slamtec_rplidar_a2'])

    rosbot_xl_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': rosbot_xl_description_urdf},
            {'use_sim_time': True},
        ],
    )

    world = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'rosbot_xl.wbt'),
        ros2_supervisor=True
    )

    rosbot_xl_ros2_control_params = os.path.join(
        package_dir, 'resource', 'rosbot_xl_controllers.yaml')

    rosbot_xl_webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'rosbot_xl'},
        output='screen',
        parameters=[
            {'robot_description': rosbot_xl_description_urdf},
            {'set_robot_state_publisher': True},
            {'use_sim_time': True},
            rosbot_xl_ros2_control_params,
        ],
        remappings=[
            ('rosbot_xl_base_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('odom', 'rosbot_xl_base_controller/odom'),
            ('rosbot_xl/laser', '/scan'),
            ('rosbot_xl/rl_range', '/range/rl'),
            ('rosbot_xl/rr_range', '/range/rr'),
            ('rosbot_xl/fl_range', '/range/fl'),
            ('rosbot_xl/fr_range', '/range/fr')
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
            'rosbot_xl_base_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    ekf_config = os.path.join(package_dir, 'resource', 'ekf.yaml')
    laser_filter_config = os.path.join(package_dir, 'resource', 'laser_filter.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True},
            {'odom0': '/' + 'rosbot_xl_base_controller/odom'}
        ]
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[laser_filter_config],
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
        rosbot_xl_webots_robot_driver,
        RegisterEventHandler(
                        event_handler=launch.event_handlers.OnProcessExit(
                            target_action=world._supervisor,
                            on_exit=[launch.actions.EmitEvent(
                                event=launch.events.Shutdown())],
                        )
        ),
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        rosbot_xl_state_publisher,
        laser_filter_node,
        robot_localization_node
    ])
