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

"""Launch the Rat's Life world and move the robot autonomously through the world to map the environment."""

import os
import json
from math import pi, sin, cos
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from geometry_msgs.msg import Quaternion


class WaypointCollection:
    """Creates a list of Navigation2 compatibile waypoints out of [x, y, theta] parameters defined for the each point."""

    def __init__(self, frame_id='odom'):
        self.__waypoints = {'poses': []}
        self.__frame_id = frame_id

    def add(self, position=None, orientation=None):
        if orientation is not None:
            q = Quaternion()
            q.z = sin(orientation / 2)
            q.w = cos(orientation / 2)
            orientation = {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}
        if position is not None:
            position = {'x': position[0], 'y': position[1], 'z': 0}
        if position is None:
            position = self.__waypoints['poses'][-1]['pose']['position']
        if orientation is None:
            orientation = self.__waypoints['poses'][-1]['pose']['orientation']

        self.__waypoints['poses'].append({
            'header': {'frame_id': self.__frame_id},
            'pose': {
                'orientation': orientation,
                'position': position
            }
        })

    def export(self):
        return json.dumps(self.__waypoints)


def get_waypoints():
    """Add a list of waypoints to `WaypointCollection` and return Navigation2 compatible JSON string."""
    collection = WaypointCollection()

    collection.add(position=[0, 0], orientation=0)      # Initial pose
    collection.add(orientation=-pi/2)                   # Explore the hall, rotate
    collection.add(orientation=0)                       # Explore the hall, rotate
    collection.add(position=[0.28, 0], orientation=0)   # Move near RED entrance
    collection.add(orientation=pi/2)                    # Rotate towards RED
    collection.add(position=[0.28, 0.2])                # Move inside RED
    collection.add(orientation=0)                       # Explore RED, rotate
    collection.add(orientation=-pi)                     # Explore RED, rotate
    collection.add(orientation=-pi/2)                   # Rotate towards BLUE
    collection.add(position=[0.23, -0.2])               # Move inside BLUE
    collection.add(orientation=0)                       # Explore BLUE, rotate
    collection.add(orientation=pi/3)                    # Explore BLUE, rotate
    collection.add(orientation=-pi/2)                   # Explore BLUE, rotate
    collection.add(orientation=-2.7*pi)                 # Explore BLUE, rotate
    collection.add(orientation=pi/2)                    # Rotate towards the hall
    collection.add(position=[0.23, 0])                  # Go to the hall
    collection.add(orientation=1/3*pi)                  # Explore the hall, rotate
    collection.add(orientation=pi)                      # Rotate towards back
    collection.add(position=[-0.1, 0])                  # Go to the GREEN entrance
    collection.add(orientation=pi/2)                    # Explore hall, rotate
    collection.add(orientation=pi/4)                    # Explore hall, rotate
    collection.add(orientation=2/3*pi)                  # Explore hall, rotate
    collection.add(orientation=-pi/2)                   # Rotate towards the GREEN
    collection.add(position=[-0.1, -0.2])               # Move inside GREEN
    collection.add(orientation=0)                       # Explore GREEN, rotate
    collection.add(orientation=pi/3)                    # Explore GREEN, rotate
    collection.add(orientation=-pi/2)                   # Explore GREEN, rotate
    collection.add(orientation=3/4*pi)                  # Explore GREEN, rotate

    return collection.export()


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_epuck')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world', default='rats_life_benchmark.wbt')

    # Webots
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world
        }.items()
    )

    # Launch Navigation2 without localization and without costmaps.
    # As the launch file is intended for mapping comparison only (real vs. physical) we want strictly to follow the waypoints
    # (without obstacle avoidance).
    # This way, mapping quality is not compromised by different paths.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', os.path.join(package_dir, 'resource', 'nav2_rats_life_waypoints.yaml'))
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
        executable='simple_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'fill_map': True}],
    )

    # Send the waypoints to Navigation2 package once the corresponding node is ready.
    send_waypoints = ExecuteProcess(
        # The goal has to be sent once the simulation is up and running.
        # Therefore, we keep sending the goal until it is accepted.
        cmd=[
            'sleep 5;'
            'while [ -z '
            f'`ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \'{get_waypoints()}\' | grep accepted`'
            ']; do'
            '  sleep 3;'
            'done'
        ],
        shell=True
    )

    return LaunchDescription([
        webots_launch,
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
