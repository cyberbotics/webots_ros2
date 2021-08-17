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

"""Test the `webots_ros2_driver` package."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_driver.py

import os
import time
import pathlib
import rclpy
from sensor_msgs.msg import Range
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_tests.utils import TestWebots


def generate_test_description():
    package_dir = get_package_share_directory('webots_ros2_tests')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'driver_test.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'driver_test.wbt'),
        gui=False,
    )

    webots_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    return LaunchDescription([
        webots_driver,
        webots,
        launch_testing.actions.ReadyToTest(),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])


class TestDriver(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node('driver_tester')
        self.wait_for_clock(self.__node)

    def testDistanceSensor(self):
        self.wait_for_messages(self.__node, Range, '/Pioneer_3_AT/so4',
                               condition=lambda msg: msg.range > 0.1 and msg.range < 1.0)

    def tearDown(self):
        self.__node.destroy_node()
