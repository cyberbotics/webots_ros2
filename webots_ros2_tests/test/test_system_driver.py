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
import pytest
import rclpy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, Image, Imu, Illuminance
from std_msgs.msg import ColorRGBA, Float32
from geometry_msgs.msg import PointStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_tests.utils import TestWebots


@pytest.mark.rostest
def generate_test_description():
    package_dir = get_package_share_directory('webots_ros2_tests')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'driver_test.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'driver_test.wbt'),
        gui=False,
        mode='fast'
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
        self.wait_for_clock(self.__node, messages_to_receive=20)

    def testRangeFinder(self):
        def on_image_received(message):
            self.assertEqual(message.height, 190)
            self.assertEqual(message.width, 320)
            return True

        self.wait_for_messages(self.__node, Image, '/Pioneer_3_AT/kinect_range', condition=on_image_received)

    def testDistanceSensor(self):
        self.wait_for_messages(self.__node, Range, '/Pioneer_3_AT/so4',
                               condition=lambda msg: msg.range > 0.1 and msg.range < 1.0)

    def testCamera(self):
        def on_image_received(message):
            self.assertEqual(message.height, 190)
            self.assertEqual(message.width, 320)
            return True

        self.wait_for_messages(self.__node, Image, '/Pioneer_3_AT/kinect_color', condition=on_image_received)

    def testPythonPluginService(self):
        client = self.__node.create_client(Trigger, 'move_forward')
        if not client.wait_for_service(timeout_sec=1.0):
            self.assertTrue(False, 'The plugin service is not found')
        request = Trigger.Request()
        response_future = client.call_async(request)

        check_start_time = time.time()
        while response_future.done() is False:
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            if time.time() - check_start_time > 5:
                self.assertTrue(False, 'The plugin service is not responding')

        response = response_future.result()
        self.assertEqual(response.success, True)

    def testLED(self):
        publisher = self.__node.create_publisher(ColorRGBA, '/Pioneer_3_AT/led', 1)
        check_start_time = time.time()
        while publisher.get_subscription_count() == 0:
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            if time.time() - check_start_time > 5:
                self.assertTrue(False, 'The LED topic doesn\'t exist')

    def testIMU(self):
        def on_message_received(message):
            self.assertEqual(message.header.frame_id, 'imu_link')

            self.assertAlmostEqual(message.orientation.x, 0.0, delta=0.01)
            self.assertAlmostEqual(message.orientation.y, 0.0, delta=0.01)
            self.assertAlmostEqual(message.orientation.z, 0.0, delta=0.01)
            self.assertAlmostEqual(message.orientation.w, 1.0, delta=0.01)

            self.assertAlmostEqual(message.angular_velocity.x, 0.0, delta=0.001)
            self.assertAlmostEqual(message.angular_velocity.y, 0.0, delta=0.001)
            self.assertAlmostEqual(message.angular_velocity.z, 0.0, delta=0.001)

            # The robot might be moving forward/backward so we don't check the linear acceleration along the x axis
            self.assertAlmostEqual(message.linear_acceleration.y, 0.0, delta=0.001)
            self.assertAlmostEqual(message.linear_acceleration.z, 9.81, delta=0.001)

            return True

        self.wait_for_messages(self.__node, Imu, '/imu', condition=on_message_received)

    def testGPS(self):
        def on_position_message_received(message):
            self.assertAlmostEqual(message.point.y, 0.0, delta=0.01)
            self.assertAlmostEqual(message.point.z, 0.0, delta=0.01)
            return True

        self.wait_for_messages(self.__node, PointStamped, '/Pioneer_3_AT/gps', condition=on_position_message_received)

        def on_velocity_message_received(message):
            self.assertAlmostEqual(message.data, 0.0, delta=0.2)
            return True

        self.wait_for_messages(self.__node, Float32, '/Pioneer_3_AT/gps/velocity', condition=on_velocity_message_received)

    def testLightSensor(self):
        self.wait_for_messages(self.__node, Illuminance, '/Pioneer_3_AT/light_sensor',
                               condition=lambda msg: msg.illuminance > 0.1)

    def tearDown(self):
        self.__node.destroy_node()
