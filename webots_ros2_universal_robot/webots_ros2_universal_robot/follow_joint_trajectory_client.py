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

"""Generic client for the FollowJointTrajectory action used for multi-robot demonstration."""

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import os
import xml.etree.ElementTree as ET
from packaging import version
from ament_index_python import get_package_share_directory


class FollowJointTrajectoryClient(Node):
    def __init__(self, name, prefix):
        super().__init__(name)
        self.__client = ActionClient(self, FollowJointTrajectory, prefix + '/follow_joint_trajectory')

        # hotfix for the new topic name in the last version of joint_trajectory_controller
        # (https://github.com/cyberbotics/webots_ros2/pull/726)
        package_xml_path = os.path.join(get_package_share_directory('joint_trajectory_controller'), "package.xml")
        package_version = version.parse(ET.parse(package_xml_path).findall("version")[0].text)
        state_topic = '/state' if package_version < version.parse('3.7.0') else '/controller_state'

        self.__state_subscriber = self.create_subscription(
            JointTrajectoryControllerState, prefix + state_topic, self.__on_state_received, 1
        )
        self.__received_states_counter = 0
        self.__remaining_iteration = 0
        self.__current_trajectory = None
        self.__get_result_future = None
        self.__send_goal_future = None

    def __on_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by action server.')
            return

        self.get_logger().info('Goal accepted by action server.')
        self.__get_result_future = goal_handle.get_result_async()
        self.__get_result_future.add_done_callback(self.__on_get_result_callback)

    def __on_get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        if self.__remaining_iteration > 0:
            self.send_goal(self.__current_trajectory, self.__remaining_iteration - 1)
        else:
            rclpy.shutdown()

    def __on_state_received(self, _):
        self.__received_states_counter += 1

    def send_goal(self, trajectory, iteration=1):
        self.get_logger().info('Waiting for action server to be ready...')

        self.__client.wait_for_server()

        # WORKAROUND: The `wait_for_server()` method reports the `joint_trajectory_controller` node is ready even though it
        # needs a bit more time to get ready to receive commands.
        while self.__received_states_counter < 1:
            rclpy.spin_once(self)

        self.__current_trajectory = trajectory
        self.__remaining_iteration = iteration - 1

        goal_message = FollowJointTrajectory.Goal()
        goal_message.trajectory.joint_names = trajectory['joint_names']
        for point in trajectory['points']:
            trajectory_point = JointTrajectoryPoint(
                positions=point['positions'],
                time_from_start=Duration(
                    sec=point['time_from_start']['sec'],
                    nanosec=point['time_from_start']['nanosec']
                )
            )
            goal_message.trajectory.points.append(trajectory_point)

        self.get_logger().info('Sending goal request...')

        self.__send_goal_future = self.__client.send_goal_async(
            goal_message
        )
        self.__send_goal_future.add_done_callback(self.__on_goal_response_callback)
