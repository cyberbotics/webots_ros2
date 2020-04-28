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
from rclpy.duration import Duration
from rclpy.time import Time

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


def interp_linear(p0, p1, t_abs):
    """Perform a linear interpolation between two trajectory points."""
    t0 = Duration.from_msg(p0.time_from_start).nanoseconds / 1e9
    t1 = Duration.from_msg(p1.time_from_start).nanoseconds / 1e9
    T = t1 - t0
    t = t_abs.nanoseconds / 1e9 - t0
    ratio = max(min((t / T), 1), 0)
    q = [0] * len(p0.positions)
    qdot = [0] * len(p0.positions)
    qddot = [0] * len(p0.positions)
    for i in list(range(len(p0.positions))):
        q[i] = (1.0 - ratio) * p0.positions[i] + ratio * p1.positions[i]
        qdot[i] = (1.0 - ratio) * p0.velocities[i] + ratio * p1.velocities[i]
        qddot[i] = (1.0 - ratio) * p0.accelerations[i] + ratio * p1.accelerations[i]
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=t_abs.to_msg())


def sample_trajectory(trajectory, t):
    """
    Sample a trajectory at time t.

    Return (q, qdot, qddot) for sampling the JointTrajectory at time t,
    the time t is the time since the trajectory was started.
    """
    # First point
    if t <= Duration(seconds=0):
        return copy.deepcopy(trajectory.points[0])
    # Last point
    if t >= Duration.from_msg(trajectory.points[-1].time_from_start):
        return copy.deepcopy(trajectory.points[-1])
    # Finds the (middle) segment containing t
    i = 0
    while Duration.from_msg(trajectory.points[i + 1].time_from_start) < t:
        i += 1
    return interp_linear(trajectory.points[i], trajectory.points[i + 1], t)


def set_position_in_limit(motor, position):
    """Set the motor position respecting its limits."""
    position = max(min(position, motor.getMaxPosition()), motor.getMinPosition())
    motor.setPosition(position)


class Trajectory():
    """Trajectory representation."""

    def __init__(self, goalHandle, startTime):
        self.jointTrajectory = goalHandle.trajectory
        self.goalTolerance = goalHandle.goal_tolerance
        self.goalHandle = goalHandle
        self.startTime = startTime
        self.lastPointSent = False
        self.id = None


class TrajectoryFollower():
    """Create and handle the action 'follow_joint_trajectory' server."""

    def __init__(self, robot, node, jointPrefix, goal_time_tolerance=None):
        self.robot = robot
        self.node = node
        self.previousTime = Time(seconds=robot.getTime())
        self.timestep = int(robot.getBasicTimeStep())
        # Parse motor and position sensors
        self.motors = {}
        self.sensors = {}
        self.position = {}
        self.velocity = {}
        for i in list(range(robot.getNumberOfDevices())):
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
        # Initialize trajectory list and action server
        self.joint_path_tolerances = [0.05] * self.numberOfMotors
        self.trajectories = []
        self.server = ActionServer(self.node, FollowJointTrajectory,
                                   'follow_joint_trajectory',
                                   execute_callback=self.update,
                                   goal_callback=self.on_goal,
                                   cancel_callback=self.on_cancel,
                                   handle_accepted_callback=self.on_goal_accepted)

    def on_goal_accepted(self, goal_handle):
        """Assign ID and execute the goal."""
        assert self.trajectories[-1].id is None
        self.trajectories[-1].id = goal_handle.goal_id
        goal_handle.execute()

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
        now = Time(seconds=self.robot.getTime())
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
            time_from_start=Duration().to_msg())
        goal_handle.trajectory.points.insert(0, point0)

        # Add this trajectory to the list if not conflicting with a current trajectory
        logger = self.node.get_logger()
        for trajectory in self.trajectories:
            if bool(set(trajectory.jointTrajectory.joint_names) &
                    set(goal_handle.trajectory.joint_names)):
                logger.info('Goal Refused: a goal sharing some joint is already running')
                return GoalResponse.REJECT
        trajectory = Trajectory(goal_handle, now)
        self.trajectories.append(trajectory)
        logger.info('Goal Accepted')
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        # stop the motors
        trajectory = None
        for trajectory in self.trajectories:
            if trajectory.id == goal_handle.goal_id:
                for name in trajectory.jointTrajectory.joint_names:
                    set_position_in_limit(self.motors[name], self.sensors[name].getValue())
                self.trajectories.remove(trajectory)
                self.node.get_logger().info('Goal Canceled')
                goal_handle.destroy()
                return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def update(self, goal_handle):
        result = FollowJointTrajectory.Result()
        while self.robot:
            # Update position and velocites
            now = Time(seconds=self.robot.getTime())
            position = {}
            velocity = {}
            timeDifference = now - self.previousTime
            for name in self.sensors.keys():
                position[name] = self.sensors[name].getValue()
                if timeDifference > Duration(seconds=0):
                    velocity[name] = (position[name] - self.position[name]) / (timeDifference.nanoseconds * 1e9)
                else:
                    velocity[name] = self.velocity[name]

            if not self.trajectories:
                self.position = position
                self.velocity = velocity
                self.previousTime = now
                break

            # Look for the trajectory associated to this goal
            trajectory = None
            for traj in self.trajectories:
                if traj.id == goal_handle.goal_id:
                    trajectory = traj
                    break
            if not trajectory:
                break
            # Apply trajectory
            lastPointStart = Duration.from_msg(trajectory.jointTrajectory.points[-1].time_from_start)
            if (now - trajectory.startTime) <= lastPointStart:
                # Sending intermediate points
                trajectory.lastPointSent = False
                setpoint = sample_trajectory(trajectory.jointTrajectory,
                                             now - trajectory.startTime)
                for name in trajectory.jointTrajectory.joint_names:
                    index = trajectory.jointTrajectory.joint_names.index(name)
                    set_position_in_limit(self.motors[name], setpoint.positions[index])
                    # Velocity control is not used on the real robot and gives
                    # bad results in the simulation
                    # self.motors[name].setVelocity(math.fabs(setpoint.velocities[index]))
            elif not trajectory.lastPointSent:
                # All intermediate points sent, sending last point to make sure we reach the goal
                trajectory.lastPointSent = True
                setpoint = sample_trajectory(trajectory.jointTrajectory, lastPointStart)
                for name in trajectory.jointTrajectory.joint_names:
                    index = trajectory.jointTrajectory.joint_names.index(name)
                    set_position_in_limit(self.motors[name], setpoint.positions[index])
                    # Velocity control is not used on the real robot and gives
                    # bad results in the simulations
                    # self.motors[name].setVelocity(math.fabs(setpoint.velocities[index]))
            else:  # Off the end
                last_point = trajectory.jointTrajectory.points[-1]
                referencePositions = []
                tolerances = [0.1] * len(last_point.positions)
                for name in trajectory.jointTrajectory.joint_names:
                    referencePositions.append(position[name])
                    for tolerance in trajectory.goalTolerance:
                        if tolerance.name == name:
                            tolerances[len(referencePositions) - 1] = tolerance.position
                            break
                position_in_tol = within_tolerance(referencePositions,
                                                   last_point.positions,
                                                   tolerances)
                if position_in_tol:
                    # The arm reached the goal => Succeeded
                    result.error_code = result.SUCCESSFUL
                    goal_handle.succeed()
                    self.trajectories.remove(trajectory)
                    self.node.get_logger().info('Goal Succeeded')
                    return result
            self.position = position
            self.velocity = velocity
            self.previousTime = now
        goal_handle.abort()
        result.error_code = result.PATH_TOLERANCE_VIOLATED
        return result
