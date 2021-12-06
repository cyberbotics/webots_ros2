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

"""Launch Webots URDF converter example."""

import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_urdf_converter')
    world = LaunchConfiguration('world')

    # Define your URDF robots here
    # Names of the robots have to match the environment variable convention
    robotsList=[
            {'name': 'robot_arm',
             'urdf_location': os.path.join(package_dir, 'resource', 'kuka_lbr_iiwa_support/urdf/model.urdf'),
             'translation': '0 0 0',
             'rotation': '0 1 0 0',
             'use_sim_time': True,
            },
        ]

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        robots=robotsList,
    )

    # Use webots.getDriversList() to get all the drivers node, the basis of the
    # webots_ros2 interface.
    return LaunchDescription(webots.getDriversList() + [DeclareLaunchArgument(
            'world',
            default_value='world.wbt',
            description='Choose one of the world files from `/webots_ros2_urdf_converter/worlds` directory'
        ),
        webots,
    ])
