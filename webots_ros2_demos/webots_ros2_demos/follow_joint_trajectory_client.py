# Copyright 1996-2019 Cyberbotics Ltd.
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


class followJointTrajectoryClient(Node):

    def __init__(self, name, actionName):
        super().__init__(name)
        self.client = ActionClient(self, FollowJointTrajectory, actionName)
        self.remainingIteration = 0
        self.currentTrajectory = None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by action server.')
            return

        self.get_logger().info('Goal accepted by action server.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback from action server.')

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        if self.remainingIteration > 0:
            self.send_goal(self.currentTrajectory, self.remainingIteration - 1)
        else:
            # Shutdown after receiving a result
            rclpy.shutdown()

    def send_goal(self, trajectory, iteration=1):
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        self.currentTrajectory = trajectory
        self.remainingIteration = iteration - 1

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = trajectory['joint_names']
        for point in trajectory['points']:
            trajectoryPoint = JointTrajectoryPoint(positions=point['positions'],
                                                   velocities=point['velocities'],
                                                   accelerations=point['accelerations'],
                                                   time_from_start=Duration(
                                                       sec=point['time_from_start']['sec'],
                                                       nanosec=point['time_from_start']['nanosec']
                                                    ))
            goal_msg.trajectory.points.append(trajectoryPoint)

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
