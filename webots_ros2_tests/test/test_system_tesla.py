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

"""Test the `webots_ros2_tesla` package."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_tesla.py

import tempfile
import time
import os
import pytest
import rclpy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    # To debug simulations in CI it is recommended to use `rosbag`.
    # All files stored to `/tmp/artifacts` are later uploaded to the CI server.
    # Therefore, make sure to store all bag files under the `/tmp/artifacts`.
    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-a',
            '-o', os.path.join(
                tempfile.gettempdir(),
                'artifacts',
                f'bag_universal_robot_multirobot_{time.strftime("%Y%m%d_%H%M%S")}'
            )
        ],
        output='screen'
    )

    tesla_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_tesla'), 'launch', 'robot_launch.py')
        )
    )

    return LaunchDescription([
        tesla_webots,
        rosbag,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTesla(TestWebots):
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

        self.wait_for_messages(self.__node, Image, '/vehicle/camera', condition=on_image_message_received)

    def testCommand(self):
        def on_cmd_message_received(message):
            # There should be a speed of 50 and steering_angle < - 0.010
            if message.speed > 45 and message.steering_angle < -0.01:
                return True
            return False

        self.wait_for_messages(self.__node, AckermannDrive, '/cmd_ackermann', condition=on_cmd_message_received)

    def tearDown(self):
        self.__node.destroy_node()
