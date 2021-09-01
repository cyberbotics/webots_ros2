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
from cartographer_ros_msgs.msg import SubmapList
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    # Webots
    turtlebot_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_turtlebot'), 'launch', 'robot_launch.py')
        )
    )
    
    # Rviz SLAM
    turtlebot_SLAM = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    #Rviz Navigation
    os.environ["TURTLEBOT3_MODEL"] = "burger"
    
    turtlebot_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true',
        'map': os.path.join(get_package_share_directory('webots_ros2_turtlebot'), 'resource', 'turtlebot3_burger_example_map.yaml')}.items(),
    )

    return LaunchDescription([
        turtlebot_webots,
        turtlebot_SLAM,
        turtlebot_navigation,
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

    def testNavigation(self):
        # Set the initial pose
        publisher = self.__node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        pose_message = PoseWithCovarianceStamped()
        pose_message.header.frame_id = "map"

        initial_point = Point()
        initial_point.x = 0.0
        initial_point.y = 0.0
        initial_point.z = 0.0
        pose_message.pose.pose.position = initial_point

        initial_orientation = Quaternion()
        initial_orientation.x = 0.0
        initial_orientation.y = 0.0
        initial_orientation.z = 0.0
        initial_orientation.w = 1.0
        pose_message.pose.pose.orientation = initial_orientation

        # Wait for Webots before sending the message
        self.wait_for_clock(self.__node)
        publisher.publish(pose_message)

        # Check if the cost map is updated -> local map for navigation is working
        def on_cost_map_message_received(message):
            return 1

        self.wait_for_messages(self.__node, OccupancyGrid, '/global_costmap/costmap',condition=on_cost_map_message_received)

    def tearDown(self):
        self.__node.destroy_node()