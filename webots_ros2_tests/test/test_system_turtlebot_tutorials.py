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

"""Test the `webots_ros2_turtlebot` package on the SLAM and Navigation tutorials."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_turtlebot_tutorials.py

import os
import pytest
import rclpy
from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()
    # If ROS_REPO is testing or ROS_DISTRO is rolling, skip test so CI succeed
    if ('ROS_REPO' in os.environ and os.environ['ROS_REPO'] == 'testing') or \
            ('ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'rolling'):
        pytest.skip('ROS_REPO testing or ROS_DISTRO rolling exit')

    # Webots
    turtlebot_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_turtlebot'), 'launch', 'robot_launch.py')
        )
    )

    # Rviz SLAM
    turtlebot_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        turtlebot_webots,
        turtlebot_slam,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTurtlebotTutorials(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node('driver_tester')
        self.wait_for_clock(self.__node, messages_to_receive=20)

    def testSLAM(self):
        from nav_msgs.msg import OccupancyGrid

        def on_map_message_received(message):
            # There should be an update the map
            for value in message.data:
                if value > -1:
                    return True
            return False

        self.wait_for_messages(self.__node, OccupancyGrid, '/map', condition=on_map_message_received)

    def tearDown(self):
        self.__node.destroy_node()
