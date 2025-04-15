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

"""Test the `webots_ros2_driver` package."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_connector_and_vacuum_gripper.py

import os
import time
import pytest
import rclpy
from math import isclose
from std_msgs.msg import Bool, Float64MultiArray
from webots_ros2_msgs.msg import IntStamped
from webots_ros2_msgs.srv import GetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from launch import LaunchDescription
import launch
import launch_testing.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    package_dir = get_package_share_directory("webots_ros2_tests")
    robot_description_path = os.path.join(
        package_dir, "resource", "connector_and_vacuum_gripper_test.urdf"
    )
    object_description_path = os.path.join(
        package_dir, "resource", "object_position.urdf.xacro"
    )
    robot_ros2_control_params = os.path.join(
        package_dir, "resource", "connector_and_vacuum_gripper_test_ros2_control.yml"
    )

    webots = WebotsLauncher(
        world=os.path.join(
            package_dir, "worlds", "connector_and_vacuum_gripper_test.wbt"
        ),
        ros2_supervisor=True,
    )

    webots_robot_driver = WebotsController(
        namespace="robot",
        robot_name="connector_and_vacuum_gripper_test_robot",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": True,
                "set_robot_state_publisher": True,
            },
            robot_ros2_control_params,
        ],
        respawn=True,
    )

    webots_connector_object_driver = WebotsController(
        namespace="connector_object",
        robot_name="connector_object",
        parameters=[
            {
                "robot_description": object_description_path,
                "xacro_mappings": ["object_name:='connector_object'"],
                "use_sim_time": True,
                "set_robot_state_publisher": True,
            }
        ],
    )

    webots_vacuum_gripper_object_driver = WebotsController(
        namespace="vacuum_gripper_object",
        robot_name="vacuum_gripper_object",
        parameters=[
            {
                "robot_description": object_description_path,
                "xacro_mappings": ["object_name:='vacuum_gripper_object'"],
                "use_sim_time": True,
                "set_robot_state_publisher": True,
            }
        ],
    )

    controller_manager_timeout = ["--controller-manager-timeout", "1000"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    robot_position_controller_spawner = Node(
        namespace="robot",
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "connector_and_vacuum_gripper_position_controller",
            "--param-file",
            robot_ros2_control_params,
        ]
        + controller_manager_timeout,
        parameters=[
            {"use_sim_time": True},
        ],
    )
    robot_joint_state_broadcaster_spawner = Node(
        namespace="robot",
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "joint_state_broadcaster",
        ]
        + controller_manager_timeout,
        parameters=[
            {"use_sim_time": True},
        ],
    )

    robot_state_publisher = Node(
        namespace="robot",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )

    connector_object_state_publisher = Node(
        namespace="connector_object",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )

    vacuum_gripper_object_state_publisher = Node(
        namespace="vacuum_gripper_object",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )

    robot_ros_control_spawners = [
        robot_position_controller_spawner,
        robot_joint_state_broadcaster_spawner,
    ]

    robot_waiting_nodes = WaitForControllerConnection(
        target_driver=webots_robot_driver, nodes_to_start=robot_ros_control_spawners
    )

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            robot_state_publisher,
            webots_robot_driver,
            connector_object_state_publisher,
            vacuum_gripper_object_state_publisher,
            webots_connector_object_driver,
            webots_vacuum_gripper_object_driver,
            robot_waiting_nodes,
            launch_testing.actions.ReadyToTest(),
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )


class TestConnectorAndVacuumGripper(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node("connector_and_vacuum_gripper_tester")
        self.joint_publisher = self.__node.create_publisher(
            Float64MultiArray,
            "/robot/connector_and_vacuum_gripper_position_controller/commands",
            1,
        )
        self.wait_for_clock(self.__node, messages_to_receive=20)

    def wait_for_position(self, index, position):

        def on_position_reached(message):
            return isclose(message.position[index], position, rel_tol=1e-3)

        self.wait_for_messages(
            self.__node,
            JointState,
            "/robot/joint_states",
            condition=on_position_reached,
        )

    def testConnector(self):

        joint_msg = Float64MultiArray()

        joint_msg.data = [-0.049, 0.0]
        joint_msg.layout.dim = []
        joint_msg.layout.data_offset = 1

        time.sleep(5)

        self.joint_publisher.publish(joint_msg)
        self.wait_for_position(0, -0.049)

        lock_publisher = self.__node.create_publisher(
            Bool,
            "/robot/connector_and_vacuum_gripper_test_robot/active_connector/lock",
            1,
        )

        lock_msg = Bool()
        lock_msg.data = True

        lock_publisher.publish(lock_msg)

        time.sleep(5)  # Give Webots time to connect the object.

        is_locked_client = self.__node.create_client(
            GetBool,
            "/robot/connector_and_vacuum_gripper_test_robot/active_connector/is_locked",
        )
        is_locked_request = GetBool.Request()
        is_locked_request.ask = True

        is_locked_future = is_locked_client.call_async(is_locked_request)
        rclpy.spin_until_future_complete(self.__node, is_locked_future)

        self.assertTrue(is_locked_future.result().value)

        joint_msg.data = [0.049, 0.0]
        self.joint_publisher.publish(joint_msg)
        self.wait_for_position(0, 0.049)

        def on_message_received(message):
            self.assertEqual(message.data, 1)  # Object must be connected, fail if not.
            return True

        self.wait_for_messages(
            self.__node,
            IntStamped,
            "/robot/connector_and_vacuum_gripper_test_robot/active_connector/presence",
            condition=on_message_received,
        )

        def on_message_received(message):
            self.assertAlmostEqual(
                message.point.z, 0.15, places=2
            )  # Has object actually been picked up?
            return True

        self.wait_for_messages(
            self.__node,
            PointStamped,
            "/connector_object/connector_object/connector_object_position",
            condition=on_message_received,
        )

        lock_msg.data = False

        lock_publisher.publish(lock_msg)

        is_locked_future = is_locked_client.call_async(is_locked_request)
        rclpy.spin_until_future_complete(self.__node, is_locked_future)

        self.assertFalse(is_locked_future.result().value)

        def on_message_received(message):
            self.assertEqual(
                message.data, 0
            )  # Object must NOT be connected, fail if not.
            return True

        time.sleep(5)

        self.wait_for_messages(
            self.__node,
            IntStamped,
            "/robot/connector_and_vacuum_gripper_test_robot/active_connector/presence",
            condition=on_message_received,
        )

        def on_message_received(message):
            self.assertAlmostEqual(
                message.point.z, 0.05, places=2
            )  # Has object been dropped?
            return True

        self.wait_for_messages(
            self.__node,
            PointStamped,
            "/connector_object/connector_object/connector_object_position",
            condition=on_message_received,
        )

    def tearDown(self):
        self.__node.destroy_node()
