# Copyright 1996-2020 Cyberbotics Ltd.
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

"""Unit tests for the examples driver."""

import os
import time
import rclpy
import unittest
import launch
import launch_testing.actions

import subprocess  ## TODO: just for debugging

from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


MESSAGE_SEND_RETRY_COUNT = 10
MESSAGE_SEND_DELAY = 0.5


def check_topic_condition(
        node, topic_class, topic_name, condition, timeout_sec=2):
    msgs_rx = []
    sub = node.create_subscription(
        topic_class,
        topic_name,
        lambda msg: msgs_rx.append(msg),
        1
    )
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
        for msg in msgs_rx:
            if condition(msg):
                node.destroy_subscription(sub)
                return True
    node.destroy_subscription(sub)
    return False


def publish_twist(node, linear_x=0.0, linear_y=0.0, angular_z=0.0):
    # Publish a message
    pub = node.create_publisher(
        Twist,
        'cmd_vel',
        1
    )
    for _ in range(MESSAGE_SEND_RETRY_COUNT):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = 0.0
        pub.publish(msg)
        time.sleep(MESSAGE_SEND_DELAY)

    # Wait a bit
    time.sleep(MESSAGE_SEND_DELAY)
    node.destroy_publisher(pub)


def generate_test_description():
    """
    Launch decription configuration.

    To run the tests you can use either `launch_test` directly as:
    $ launch_test src/webots_ros2/webots_ros2_examples/test/test_runtime.py
    or `colcon`:
    $ colcon test --packages-select webots_ros2_examples
    The testing procedure is based on `launch_test`:
    https://github.com/ros2/launch/tree/master/launch_testing
    and the following example:
    https://github.com/ros2/launch_ros/blob/master/launch_testing_ros/test/examples/talker_listener_launch_test.py.
    """
    # Webots
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('webots_ros2_examples'), 'worlds', 'ros_example.wbt'),
        gui=False
    )

    # Controller node
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    controller = ControllerLauncher(package='webots_ros2_examples',
                                    node_executable='example_controller',
                                    parameters=[{'synchronization': synchronization}],
                                    output='screen')
    return launch.LaunchDescription([
        webots,
        controller,
        # Shutdown launch when Webots exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('examples_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_forward_velocity(self):
        publish_twist(self.node, linear_x=0.02)

    def test_distance_sensors(self):
        print(subprocess.check_output(['ps', '-e']))
        print('---------------')
        condition = check_topic_condition(
            self.node,
            Float64,
            'sensor',
            lambda msg: abs(msg.data) < 1E-3,
            300)
        print(subprocess.check_output(['ps', '-e']))
        self.assertTrue(
            condition, 'The node hasn\'t published any distance measurement')
