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

"""Generic client for the FollowJointTrajectory action."""

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class FollowJointTrajectoryClient(Node):

    def __init__(self, name, actionName):
        super().__init__(name)
        self.__client = ActionClient(self, FollowJointTrajectory, actionName)
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

    def __on_feedback_callback(self, _):
        self.get_logger().info('Received feedback from action server.')

    def __on_get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        if self.__remaining_iteration > 0:
            self.send_goal(self.__current_trajectory, self.__remaining_iteration - 1)
        else:
            # Shutdown after receiving a result
            rclpy.shutdown()

    def send_goal(self, trajectory, iteration=1):
        self.get_logger().info('Waiting for action server...')
        self.__client.wait_for_server()

        self.__current_trajectory = trajectory
        self.__remaining_iteration = iteration - 1

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = trajectory['joint_names']
        for point in trajectory['points']:
            trajectory_point = JointTrajectoryPoint(
                positions=point['positions'],
                velocities=point['velocities'],
                accelerations=point['accelerations'],
                time_from_start=Duration(
                    sec=point['time_from_start']['sec'],
                    nanosec=point['time_from_start']['nanosec']
                )
            )
            goal_msg.trajectory.points.append(trajectory_point)

        self.get_logger().info('Sending goal request...')

        self.__send_goal_future = self.__client.send_goal_async(
            goal_msg,
            feedback_callback=self.__on_feedback_callback
        )
        self.__send_goal_future.add_done_callback(self.__on_goal_response_callback)
