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
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan

from cartographer_ros_msgs.msg import SubmapList

from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    turtlebot_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_turtlebot'), 'launch', 'robot_launch.py')
        )
    )
    
    turtlebot_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        turtlebot_webots,
        turtlebot_cartographer,
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
        def on_map_message_received(message):
            # There should be an update of the submap
            update_found = 0
            for submap in message.submap:
                if submap.submap_version > 1:
                    update_found = 1
            return update_found

        self.wait_for_messages(self.__node, SubmapList, '/submap_list', condition=on_map_message_received)

    def tearDown(self):
        self.__node.destroy_node()

'''
    def testMovement(self):
        publisher = self.__node.create_publisher(Twist, '/cmd_vel', 1)

        def on_position_message_received(message):
            twist_message = Twist()
            twist_message.linear.x = 0.1
            publisher.publish(twist_message)

            return message.point.x > 6.7

        self.wait_for_messages(self.__node, PointStamped, '/TurtleBot3Burger/gps', condition=on_position_message_received)

    def testLidar(self):
        def on_message_received(message):
            # There should be some unreachable points and some above 0.1m
            number_of_infinities = 0
            number_of_reachables = 0
            for value in message.ranges:
                if value == float('inf'):
                    number_of_infinities += 1
                elif value > 0.1:
                    number_of_reachables += 1
            return number_of_infinities > 0 and number_of_reachables > 0

        self.wait_for_messages(self.__node, LaserScan, '/scan', condition=on_message_received)
'''