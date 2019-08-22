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

"""Implementation of the 'follow_joint_trajectory' ROS action."""

import copy
import math
import sys

from webots_ros2_core.utils import append_webots_python_lib_to_path

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from rclpy.action import ActionServer, CancelResponse, GoalResponse

try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


def trajectory_is_finite(trajectory):
    """Check if trajectory contains infinite or NaN value."""
    for point in trajectory.points:
        for position in point.positions:
            if math.isinf(position) or math.isnan(position):
                return False
        for velocity in point.velocities:
            if math.isinf(velocity) or math.isnan(velocity):
                return False
    return True


def has_velocities(trajectory):
    """Check that velocities are defined for this trajectory."""
    for point in trajectory.points:
        if len(point.velocities) != len(point.positions):
            return False
    return True


def within_tolerance(a_vec, b_vec, tol_vec):
    """Check if two vectors are equals with a given tolerance."""
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True


def interp_cubic(p0, p1, t_abs):
    """Perform a cubic interpolation between two trajectory points."""
    t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1.0e-6
    t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1.0e-6
    T = t1 - t0
    t = t_abs - t0
    q = [0] * len(p0.positions)
    qdot = [0] * len(p0.positions)
    qddot = [0] * len(p0.positions)
    for i in range(len(p0.positions)):
        a = p0.positions[i]
        b = p0.velocities[i]
        c = (-3 * p0.positions[i] + 3 * p1.positions[i] - 2 * T * p0.velocities[i] -
             T * p1.velocities[i]) / T**2
        d = (2 * p0.positions[i] - 2 * p1.positions[i] + T * p0.velocities[i] +
             T * p1.velocities[i]) / T**3

        q[i] = a + b * t + c * t**2 + d * t**3
        qdot[i] = b + 2 * c * t + 3 * d * t**2
        qddot[i] = 2 * c + 6 * d * t
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot,
                                time_from_start=Duration(sec=int(t_abs), nanosec=int(int(t_abs) *
                                                                                     1.0e+6)))


def sample_trajectory(trajectory, t):
    """
    Sample a trajectory at time t.

    Return (q, qdot, qddot) for sampling the JointTrajectory at time t,
    the time t is the time since the trajectory was started.
    """
    # First point
    if t <= 0.0:
        return copy.deepcopy(trajectory.points[0])
    # Last point
    if t >= (trajectory.points[-1].time_from_start.sec +
             trajectory.points[-1].time_from_start.nanosec * 1.0e-6):
        return copy.deepcopy(trajectory.points[-1])
    # Finds the (middle) segment containing t
    i = 0
    while (trajectory.points[i + 1].time_from_start.sec +
           trajectory.points[i + 1].time_from_start.nanosec * 1.0e-6) < t:
        i += 1
    return interp_cubic(trajectory.points[i], trajectory.points[i + 1], t)


class TrajectoryFollower():
    """Create and handle the action 'follow_joint_trajectory' server."""

    def __init__(self, robot, node, jointPrefix, goal_time_tolerance=None):
        self.robot = robot
        self.node = node
        self.previousTime = robot.getTime()
        self.timestep = int(robot.getBasicTimeStep())
        # Parse motor and position sensors
        self.motors = {}
        self.sensors = {}
        self.position = {}
        self.velocity = {}
        for i in range(robot.getNumberOfDevices()):
            device = robot.getDeviceByIndex(i)
            if device.getNodeType() in [Node.LINEAR_MOTOR, Node.ROTATIONAL_MOTOR]:
                name = device.getName()
                positionSensor = device.getPositionSensor()
                if positionSensor is None:
                    self.node.get_logger().info('Motor "%s" doesn\'t have any position \
                        sensor, impossible to include it in the action server.')
                else:
                    self.motors[jointPrefix + name] = device
                    self.sensors[jointPrefix + name] = positionSensor
                    self.position[jointPrefix + name] = 0.0
                    self.velocity[jointPrefix + name] = 0.0
                    positionSensor.enable(self.timestep)
        self.numberOfMotors = len(self.motors)
        # Initialize tarjectories and action server
        self.goal_handle = None
        self.last_point_sent = True
        self.joint_goal_tolerances = [0.05] * self.numberOfMotors
        self.trajectory = None
        self.server = ActionServer(self.node, FollowJointTrajectory,
                                   'follow_joint_trajectory',
                                   execute_callback=self.update,
                                   goal_callback=self.on_goal,
                                   cancel_callback=self.on_cancel)

    def on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        # Checks if the joints are just incorrect
        for name in goal_handle.trajectory.joint_names:
            if name not in self.motors.keys():
                self.node.get_logger().warn('Received a goal with incorrect joint names: (%s)' %
                                            ', '.join(goal_handle.trajectory.joint_names))
                return GoalResponse.REJECT
        if not trajectory_is_finite(goal_handle.trajectory):
            self.node.get_logger().warn('Received a goal with infinites or NaNs')
            return GoalResponse.REJECT

        # Checks that the trajectory has velocities
        if not has_velocities(goal_handle.trajectory):
            self.node.get_logger().warn('Received a goal without velocities')
            return GoalResponse.REJECT

        # Inserts the current setpoint at the head of the trajectory
        now = self.robot.getTime()
        positions = []
        velocities = []
        accelerations = []
        for name in goal_handle.trajectory.joint_names:
            positions.append(self.position[name])
            velocities.append(self.velocity[name])
            accelerations.append(0.0)
        point0 = JointTrajectoryPoint(
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
            time_from_start=Duration())
        goal_handle.trajectory.points.insert(0, point0)
        self.trajectory_t0 = now

        # Replaces the goal
        self.goal_handle = goal_handle
        self.trajectory = goal_handle.trajectory
        self.last_point_sent = False
        self.node.get_logger().info('Goal Accepted')
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        # stop the motors
        for name in TrajectoryFollower.jointNames:
            self.motors[name].setPosition(self.sensors[name].getValue())
        self.goal_handle = None
        self.last_point_sent = True
        self.node.get_logger().info('Goal Canceled')
        return CancelResponse.ACCEPT

    def update(self, goal_handle):
        result = FollowJointTrajectory.Result()
        while self.robot:
            # Update position and velocites
            now = self.robot.getTime()
            position = {}
            velocity = {}
            timeDifference = now - self.previousTime
            for name in self.sensors.keys():
                position[name] = self.sensors[name].getValue()
                if timeDifference > 0.0:
                    velocity[name] = (position[name] - self.position[name]) / timeDifference
                else:
                    velocity[name] = self.velocity[name]

            if not self.trajectory:
                self.position = position
                self.velocity = velocity
                self.previousTime = now
                continue

            # Apply trajectory
            if (now - self.trajectory_t0) <= (self.trajectory.points[-1].time_from_start.sec +
                                              self.trajectory.points[-1].time_from_start.nanosec *
                                              1.0e-6):
                # Sending intermediate points
                self.last_point_sent = False
                setpoint = sample_trajectory(self.trajectory, now - self.trajectory_t0)
                for name in self.trajectory.joint_names:
                    index = self.trajectory.joint_names.index(name)
                    self.motors[name].setPosition(setpoint.positions[index])
                    # Velocity control is not used on the real robot and gives
                    # bad results in the simulation
                    # self.motors[name].setVelocity(math.fabs(setpoint.velocities[index]))
            elif not self.last_point_sent:
                # All intermediate points sent, sending last point to make sure we reach the goal
                self.last_point_sent = True
                last_point = self.trajectory.points[-1]
                position_in_tol = within_tolerance(position, last_point.positions,
                                                   self.joint_goal_tolerances)
                setpoint = sample_trajectory(self.trajectory,
                                             self.trajectory.points[-1].time_from_start.sec +
                                             self.trajectory.points[-1].time_from_start.nanosec *
                                             1.0e-6)
                for name in self.trajectory.joint_names:
                    index = self.trajectory.joint_names.index(name)
                    self.motors[name].setPosition(setpoint.positions[index])
                    # Velocity control is not used on the real robot and gives
                    # bad results in the simulation
                    # self.motors[name].setVelocity(math.fabs(setpoint.velocities[index]))
            else:  # Off the end
                if self.goal_handle:
                    last_point = self.trajectory.points[-1]
                    position_in_tol = within_tolerance(position, last_point.positions,
                                                       [0.1] * self.numberOfMotors)
                    velocity_in_tol = within_tolerance(velocity, last_point.velocities,
                                                       [0.05] * self.numberOfMotors)
                    if position_in_tol and velocity_in_tol:
                        # The arm reached the goal (and isn't moving) => Succeeded
                        result.error_code = result.SUCCESSFUL
                        goal_handle.succeed()
                        self.goal_handle = None
                        self.node.get_logger().info('Goal Succeeded')
                        return result
            self.position = position
            self.velocity = velocity
            self.previousTime = now
        result.error_code = result.PATH_TOLERANCE_VIOLATED
        return result
