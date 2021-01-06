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

"""Launch Rat's Life world with navigation."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    package_dir = get_package_share_directory('webots_ros2_epuck')
    nav2_map = os.path.join(package_dir, 'resource', 'map_rats_life.yaml')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'webots_ros2_epuck'),
            ('executable', 'driver'),
            ('world', os.path.join(package_dir, 'worlds', 'rats_life_benchmark.wbt')),
        ],
        condition=launch.conditions.IfCondition(use_sim_time)
    )

    # Launch complete Navigation2 with `amcl` (particle filter to track the pose of a robot)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('map', nav2_map),
            ('use_sim_time', use_sim_time),
            ('params_file', os.path.join(package_dir, 'resource', 'nav2_params.yaml'))
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Set initial position of the robot within the provided map.
    # The initial position can be also be set in RViz2 menu `2D Pose Estimate`.
    initial_position = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            '/initialpose',
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

    return LaunchDescription([
        webots,
        nav2,
        rviz,
        initial_position,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock if true'
        )
    ])
