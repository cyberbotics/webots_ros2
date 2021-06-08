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

"""Implementation of the 'follow_joint_trajectory' ROS action."""

import math
import time
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from webots_ros2_core.webots.controller import Node


def to_s(duration):
    return Duration.from_msg(duration).nanoseconds / 1e9


class TrajectoryFollower:
    """Create and handle the action 'follow_joint_trajectory' server."""

    def __init__(self, robot, node, joint_prefix, controller_name):
        self.__robot = robot
        self.__node = node
        self.__timestep = int(robot.getBasicTimeStep())

        # Config
        self.__default_tolerance = 0.05

        # Parse motor and position sensors
        self.__motors = {}
        for i in list(range(robot.getNumberOfDevices())):
            device = robot.getDeviceByIndex(i)
            if device.getNodeType() in [Node.LINEAR_MOTOR, Node.ROTATIONAL_MOTOR]:
                name = device.getName()
                if device.getPositionSensor() is None:
                    self.__node.get_logger().warn(f'Motor `{name}` doesn\'t have any position sensor.')
                else:
                    self.__motors[joint_prefix + name] = device
                    device.getPositionSensor().enable(self.__timestep)

        # Initialize trajectory list and action server
        self.__current_point_index = 1
        self.__start_time = None
        self.__goal = None
        self.__mode = None
        self.__tolerances = {}
        self.__server = ActionServer(
            self.__node,
            FollowJointTrajectory,
            controller_name + '/follow_joint_trajectory',
            execute_callback=self.__on_update,
            goal_callback=self.__on_goal,
            cancel_callback=self.__on_cancel,
            handle_accepted_callback=self.__on_goal_accepted
        )

    def log(self, *args):
        self.__node.get_logger().warn(' '.join([str(arg) for arg in args]))

    def __on_goal_accepted(self, goal_handle):
        goal_handle.execute()

    def __on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        # Reject if joints don't match
        for name in goal_handle.trajectory.joint_names:
            if name not in self.__motors.keys():
                joint_names = ', '.join(goal_handle.trajectory.joint_names)
                self.__node.get_logger().error(f'Received a goal with incorrect joint names: ({joint_names})')
                return GoalResponse.REJECT

        # Reject if infinity or NaN
        for point in goal_handle.trajectory.points:
            for position, velocity in zip(point.positions, point.velocities):
                if math.isinf(position) or math.isnan(position) or math.isinf(velocity) or math.isnan(velocity):
                    self.__node.get_logger().error('Received a goal with infinites or NaNs')
                    return GoalResponse.REJECT

        # Reject if joints are already controlled
        if self.__goal is not None:
            self.__node.get_logger().error('Cannot accept multiple goals')
            return GoalResponse.REJECT

        # Store goal data
        self.__goal = goal_handle
        self.__current_point_index = 1
        self.__start_time = self.__robot.getTime()

        for tolerance in self.__goal.goal_tolerance:
            self.__tolerances[tolerance.name] = tolerance.position
        for name in self.__goal.trajectory.joint_names:
            if name not in self.__tolerances.keys():
                self.__tolerances[name] = self.__default_tolerance

        self.__mode = 'velocity'
        for point in self.__goal.trajectory.points:
            if to_s(point.time_from_start) != 0:
                self.__mode = 'time'
                break

        # If a user forget the initial position
        if to_s(self.__goal.trajectory.points[0].time_from_start) != 0:
            initial_point = JointTrajectoryPoint(
                positions=[self.__motors[name].getPositionSensor().getValue() for name in self.__goal.trajectory.joint_names],
                time_from_start=Duration().to_msg()
            )
            self.__goal.trajectory.points.insert(0, initial_point)

        # Accept the trajectory
        self.__node.get_logger().info('Goal Accepted')
        return GoalResponse.ACCEPT

    def __on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        if self.__goal is not None:
            # Stop motors
            for name in self.__goal.trajectory.joint_names:
                motor = self.__motors[name]
                motor.setPosition(motor.getPositionSensor().getValue())

            self.__goal = None
            self.__node.get_logger().info('Goal Canceled')
            goal_handle.destroy()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def __regulate_velocity_mode(self):
        curr_point = self.__goal.trajectory.points[self.__current_point_index]

        for index, name in enumerate(self.__goal.trajectory.joint_names):
            self.__set_motor_position(name, curr_point.positions[index])

        done = TrajectoryFollower.__is_within_tolerance(
            curr_point.positions,
            [self.__motors[name].getPositionSensor().getValue() for name in self.__goal.trajectory.joint_names],
            [self.__tolerances[name] for name in self.__goal.trajectory.joint_names],
        )
        if done:
            self.__current_point_index += 1
            if self.__current_point_index >= len(self.__goal.trajectory.points):
                return True
        return False

    def __regulate_time_mode(self):
        now = self.__robot.getTime()
        prev_point = self.__goal.trajectory.points[self.__current_point_index - 1]
        curr_point = self.__goal.trajectory.points[self.__current_point_index]
        time_passed = now - self.__start_time

        if time_passed <= to_s(curr_point.time_from_start):
            # Linear interpolation
            ratio = (time_passed - to_s(prev_point.time_from_start)) /\
                (to_s(curr_point.time_from_start) - to_s(prev_point.time_from_start))
            for index, name in enumerate(self.__goal.trajectory.joint_names):
                side = -1 if curr_point.positions[index] < prev_point.positions[index] else 1
                target_position = prev_point.positions[index] + \
                    side * ratio * abs(curr_point.positions[index] - prev_point.positions[index])
                self.__set_motor_position(name, target_position)
        else:
            self.__current_point_index += 1
            if self.__current_point_index >= len(self.__goal.trajectory.points):
                return True
        return False

    def __set_motor_position(self, name, target_position):
        target_position = min(max(target_position, self.__motors[name].getMinPosition()), self.__motors[name].getMaxPosition())
        self.__motors[name].setPosition(target_position)

    async def __on_update(self, goal_handle):
        feedback_message = FollowJointTrajectory.Feedback()
        feedback_message.joint_names = list(self.__goal.trajectory.joint_names)

        while self.__goal and self.__robot:
            done = False

            # Regulate
            if self.__mode == 'time':
                done = self.__regulate_time_mode()
            else:
                done = self.__regulate_velocity_mode()

            # Finalize
            if done:
                self.__node.get_logger().info('Goal Succeeded')
                self.__goal = None
                goal_handle.succeed()
                return FollowJointTrajectory.Result()

            # Publish state
            time_passed = self.__robot.getTime() - self.__start_time
            feedback_message.actual.positions = [self.__motors[name].getPositionSensor().getValue()
                                                 for name in self.__goal.trajectory.joint_names]
            feedback_message.actual.time_from_start = Duration(seconds=time_passed).to_msg()
            goal_handle.publish_feedback(feedback_message)

            time.sleep(self.__timestep * 1e-3)

        result = FollowJointTrajectory.Result()
        result.error_code = result.PATH_TOLERANCE_VIOLATED
        return result

    @staticmethod
    def __is_within_tolerance(a_vec, b_vec, tol_vec):
        """Check if two vectors are equals with a given tolerance."""
        for a, b, tol in zip(a_vec, b_vec, tol_vec):
            if abs(a - b) > tol:
                return False
        return True
