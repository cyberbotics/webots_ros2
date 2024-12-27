#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
# Copyright 2023 Husarion
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

"""Test the `webots_ros2_husarion` package."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_rosbot_xl.py

import os
import math
import pytest
import rclpy
from nav_msgs.msg import Odometry
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    os.environ["USERNAME"] = "root"
    initialize_webots_test()
    # If ROS_DISTRO is rolling, skip the test as some required packages are missing (cf. ci_after_init.bash)
    if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'rolling':
        pytest.skip('ROS_DISTRO is rolling, skipping this test')

    rosbot_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_husarion'), 'launch', 'rosbot_xl_launch.py'),
        )
    )

    return LaunchDescription([
        rosbot_webots,
        launch_testing.actions.ReadyToTest(),
    ])


class TestROSbotXL(TestWebots):
    @classmethod
    def setUpClass(cls):
        os.environ["USERNAME"] = "root"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node('driver_tester')

    def testMovement(self):
        use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])

        publisher = None
        if use_twist_stamped:
            publisher = self.__node.create_publisher(TwistStamped, '/cmd_vel', 1)

            def on_position_message_received(message):
                twist_message = TwistStamped()
                twist_message.header.stamp = self.__node.get_clock().now().to_msg()
                twist_message.twist.linear.x = 0.5
                twist_message.twist.angular.z = 0.3
                publisher.publish(twist_message)

                if message.pose.pose.position.x > 0.5 and message.pose.pose.orientation.w < 0.9:
                    return True
                return False
        else:
            publisher = self.__node.create_publisher(Twist, '/cmd_vel', 1)

            def on_position_message_received(message):
                twist_message = Twist()
                twist_message.linear.x = 0.5
                twist_message.angular.z = 0.3
                publisher.publish(twist_message)

                if message.pose.pose.position.x > 0.5 and message.pose.pose.orientation.w < 0.9:
                    return True
                return False

        self.wait_for_messages(self.__node, Odometry, '/odometry/filtered', condition=on_position_message_received)

    def testScan(self):
        def on_scan_message_received(message):
            # There should be at least 1 range bigger than 0 and some = 0
            number_of_inf = 0
            number_of_non_zeroes = 0
            for value in message.ranges:
                if value == float('inf') or math.isnan(value):
                    number_of_inf += 1
                elif value > 0.:
                    number_of_non_zeroes += 1
            return number_of_inf > 0 and number_of_non_zeroes > 0

        self.wait_for_messages(self.__node, LaserScan, '/scan_filtered', condition=on_scan_message_received)

    def tearDown(self):
        self.__node.destroy_node()
