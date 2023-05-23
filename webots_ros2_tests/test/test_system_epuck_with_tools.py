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

"""Test the `webots_ros2_epuck` package on the extra tools."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_epuck_with_tools.py

import os
import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from rclpy.action import ActionClient
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()
    # If ROS_DISTRO is rolling, skip the test as some required packages are missing (cf. ci_after_init.bash)
    if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'rolling':
        pytest.skip('ROS_DISTRO is rolling, skipping this test')

    epuck_with_tools_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_epuck'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={'nav': 'true'}.items(),
    )

    return LaunchDescription([
        epuck_with_tools_webots,
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

    def testMap(self):
        def on_map_message_received(message):
            # Should be an update of the map
            for value in message.data:
                if value > -1:
                    return True
            return False

        self.wait_for_messages(self.__node, OccupancyGrid, '/map', condition=on_map_message_received)

    def testMovement(self):
        from nav2_msgs.action import NavigateToPose

        # Delay before publishing goal position (navigation initialization can be long in the CI)
        goal_action = ActionClient(self.__node, NavigateToPose, 'navigate_to_pose')
        goal_message = NavigateToPose.Goal()
        goal_message.pose.header.stamp = self.__node.get_clock().now().to_msg()
        goal_message.pose.header.frame_id = 'map'
        goal_message.pose.pose.position.x = 0.37
        goal_message.pose.pose.position.y = 0.2
        goal_message.pose.pose.orientation.z = 0.544
        goal_message.pose.pose.orientation.w = 0.838
        goal_action.wait_for_server()
        self.__node.get_logger().info('Server is ready, waiting 10 seconds to send goal position.')
        self.wait_for_clock(self.__node, messages_to_receive=1000)
        goal_action.send_goal_async(goal_message)
        self.__node.get_logger().info('Goal position sent.')

        def on_message_received(message):
            return message.pose.pose.position.x > 0.25

        self.wait_for_messages(self.__node, Odometry, '/odom', condition=on_message_received, timeout=60*5)

    def tearDown(self):
        self.__node.destroy_node()
