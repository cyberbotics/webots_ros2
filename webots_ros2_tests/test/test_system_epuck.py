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

"""Test the `webots_ros2_epuck` package."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_epuck.py

import os
import pytest
import rclpy
from sensor_msgs.msg import Range, Image, LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    epuck_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_epuck'), 'launch', 'robot_launch.py')
        )
    )

    return LaunchDescription([
        epuck_webots,
        launch_testing.actions.ReadyToTest(),
    ])


class TestEpuck(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node('driver_tester')
        self.wait_for_clock(self.__node, messages_to_receive=20)

    def testCamera(self):
        def on_image_message_received(message):
            # There should be data values between 0 and 255 included
            if message.data[0] > 0 and message.data[0] < 255:
                return True
            return False

        self.wait_for_messages(self.__node, Image, '/camera/image_color', condition=on_image_message_received)

    def testPs0(self):
        def on_range_message_received(message):
            # There should be a range bigger 0 and 2
            if message.range >= 0. and message.range <= 2.:
                return True
            return False

        self.wait_for_messages(self.__node, Range, '/ps0', condition=on_range_message_received)

    def testToF(self):
        def on_range_message_received(message):
            # There should be a range bigger 0 and 2
            if message.range >= 0. and message.range <= 2.:
                return True
            return False

        self.wait_for_messages(self.__node, Range, '/tof', condition=on_range_message_received)

    def testMovement(self):
        use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])

        if use_twist_stamped:
            publisher = self.__node.create_publisher(TwistStamped, '/cmd_vel', 1)

            def on_position_message_received(message):
                twist_message = TwistStamped()
                twist_message.header.stamp = self.__node.get_clock().now().to_msg()
                twist_message.twist.linear.x = 0.1
                publisher.publish(twist_message)

                # E_puck should move forward
                if message.pose.pose.position.x > 0.5:
                    return True
                return False

        else:
            publisher = self.__node.create_publisher(Twist, '/cmd_vel', 1)

            def on_position_message_received(message):
                twist_message = Twist()
                twist_message.linear.x = 0.1
                publisher.publish(twist_message)

                # E_puck should move forward
                if message.pose.pose.position.x > 0.5:
                    return True
                return False

        self.wait_for_messages(self.__node, Odometry, '/odom', condition=on_position_message_received)

    def testScan(self):
        def on_scan_message_received(message):
            # There should be at least 1 range bigger than 0 and some = 0
            number_of_zeroes = 0
            number_of_non_zeroes = 0
            for value in message.ranges:
                if value == 0.:
                    number_of_zeroes += 1
                elif value > 0.:
                    number_of_non_zeroes += 1
            return number_of_zeroes > 0 and number_of_non_zeroes > 0

        self.wait_for_messages(self.__node, LaserScan, '/scan', condition=on_scan_message_received)

    def tearDown(self):
        self.__node.destroy_node()
