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

"""Test the `webots_ros2_universal_robot` package."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_universal_robot.py

import os
import time
import tempfile
import pytest
import rclpy
import launch_testing.actions
from sensor_msgs.msg import Range, JointState
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_universal_robot'), 'launch', 'multirobot_launch.py')
        )
    )

    # To debug simulations in CI it is recommended to use `rosbag`.
    # All files stored to `/tmp/artifacts` are later uploaded to the CI server.
    # Therefore, make sure to store all bag files under the `/tmp/artifacts`.
    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-a', '--include-hidden-topics',
            '-o', os.path.join(
                tempfile.gettempdir(),
                'artifacts',
                f'bag_universal_robot_multirobot_{time.strftime("%Y%m%d_%H%M%S")}'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        rosbag,
        launch_testing.actions.ReadyToTest(),
    ])


class TestUniversalRobot(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node('driver_tester')
        self.wait_for_clock(self.__node, messages_to_receive=20)

    def testAbbCaughtCan(self):
        # The robot should catch the can in the simulation.
        self.wait_for_messages(self.__node, Range, '/abb/abbirb4600/object_present_sensor', timeout=200,
                               condition=lambda message: message.range < 0.07)

    def testUr5eJointStates(self):
        # The robot should have been spawned and send JointState.
        def on_joint_state_message_received(message):
            # There should be at least one position greater than 0
            for position in message.position:
                if position > 0.0:
                    return True
            return False

        self.wait_for_messages(self.__node, JointState, '/ur5e/joint_states', timeout=200,
                               condition=on_joint_state_message_received)

    def tearDown(self):
        self.__node.destroy_node()
