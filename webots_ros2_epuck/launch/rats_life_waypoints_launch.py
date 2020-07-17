#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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


WAYPOINTS = '''
poses:
# Move near RED entrance
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: 1

# Rotate towards RED
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0.707
      w: 0.707

# Move inside RED
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0.707
      w: 0.707

# Explore RED, rotate
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: 1

# Explore RED, rotate
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: 1
      w: 0

# Rotate towards BLUE
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: -0.707
      w: 0.707

# Move inside BLUE
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: -0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: -0.707
      w: 0.707

# Explore BLUE, rotate
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: -0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: 1

# Explore BLUE, rotate
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: -0.23
      z: 0
    orientation:
      x: 0
      y: 0
      z: 1
      w: 0

# Rotate towards the hall
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: -0.23
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0.707
      w: 0.707

# Go to the hall
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0.707
      w: 0.707

# Rotate towards back
- header:
    frame_id: odom
  pose:
    position:
      x: 0.26
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 1
      w: 0

# Go to the GREEN entrance
- header:
    frame_id: odom
  pose:
    position:
      x: -0.1
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: -1

# Rotate towards GREEN
- header:
    frame_id: odom
  pose:
    position:
      x: -0.1
      y: 0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: -0.707
      w: 0.707

# Move inside GREEN
- header:
    frame_id: odom
  pose:
    position:
      x: -0.1
      y: -0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: -0.707
      w: 0.707

# Explore GREEN, rotate
- header:
    frame_id: odom
  pose:
    position:
      x: -0.1
      y: -0.2
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: 1

# Explore GREEN, rotate
- header:
    frame_id: odom
  pose:
    position:
      x: -0.1
      y: -0.23
      z: 0
    orientation:
      x: 0
      y: 0
      z: 1
      w: 0
'''


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    package_dir = get_package_share_directory('webots_ros2_epuck')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
        launch_arguments=[
            ('map_subscribe_transient_local', 'true'),
            ('use_sim_time', use_sim_time),
            ('params_file', os.path.join(package_dir, 'resource', 'nav2_params.yaml'))
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'resource', 'all.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    mapper = Node(
        package='webots_ros2_epuck',
        node_executable='simple_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'fill_map': True}],
    )

    send_waypoints = ExecuteProcess(
        cmd=[f'sleep 6 && ros2 action send_goal /FollowWaypoints nav2_msgs/action/FollowWaypoints "{WAYPOINTS}"'],
        shell=True
    )

    return LaunchDescription([
        webots,
        nav2,
        rviz,
        mapper,
        send_waypoints,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock if true'
        )
    ])
