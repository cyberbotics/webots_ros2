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

"""universal_robot_ros controller."""

import rclpy
import os
import sys
import copy
import math


if not 'WEBOTS_HOME' in os.environ:
    sys.exit('"WEBOTS_HOME" not defined.')
sys.path.append(os.environ['WEBOTS_HOME'] + '/lib/python%d%d' % (sys.version_info[0], sys.version_info[1]))
from controller import Robot

from webots_ros2.joint_state_publisher import JointStatePublisher

from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor


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


def reorder_trajectory_joints(trajectory, joint_names):
    """Reorder the trajectory points according to the order in joint_names."""
    order = [trajectory.joint_names.index(j) for j in joint_names]
    new_points = []
    for point in trajectory.points:
        new_points.append(JointTrajectoryPoint(
            positions=[point.positions[i] for i in order],
            velocities=[point.velocities[i] for i in order] if point.velocities else [],
            accelerations=[point.accelerations[i] for i in order] if point.accelerations else [],
            time_from_start=point.time_from_start))
    trajectory.joint_names = joint_names
    trajectory.points = new_points


def within_tolerance(a_vec, b_vec, tol_vec):
    """Check if two vectors are equals with a given tolerance."""
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True


def interp_cubic(p0, p1, t_abs):
    """Perform a cubic interpolation between two trajectory points."""
    T = (p1.time_from_start - p0.time_from_start).to_sec()
    t = t_abs - (p0.time_from_start.sec + p0.time_from_start.nanosec * 1.0e-6)
    q = [0] * 6
    qdot = [0] * 6
    qddot = [0] * 6
    for i in range(len(p0.positions)):
        a = p0.positions[i]
        b = p0.velocities[i]
        c = (-3 * p0.positions[i] + 3 * p1.positions[i] - 2 * T * p0.velocities[i] - T * p1.velocities[i]) / T**2
        d = (2 * p0.positions[i] - 2 * p1.positions[i] + T * p0.velocities[i] + T * p1.velocities[i]) / T**3

        q[i] = a + b * t + c * t**2 + d * t**3
        qdot[i] = b + 2 * c * t + 3 * d * t**2
        qddot[i] = 2 * c + 6 * d * t
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=Duration(seconds=t_abs))


def sample_trajectory(trajectory, t):
    """Return (q, qdot, qddot) for sampling the JointTrajectory at time t,
       the time t is the time since the trajectory was started."""
    # First point
    if t <= 0.0:
        return copy.deepcopy(trajectory.points[0])
    # Last point
    if t >= (trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1.0e-6):
        return copy.deepcopy(trajectory.points[-1])
    # Finds the (middle) segment containing t
    i = 0
    while (trajectory.points[i + 1].time_from_start.sec + trajectory.points[i + 1].time_from_start.nanosec * 1.0e-6) < t:
        i += 1
    return interp_cubic(trajectory.points[i], trajectory.points[i + 1], t)


class TrajectoryFollower(object):
    """Create and handle the action 'follow_joint_trajectory' server."""

    jointNames = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    def __init__(self, robot, node, jointStatePublisher, jointPrefix, goal_time_tolerance=None):
        self.robot = robot
        self.node = node
        self.jointPrefix = jointPrefix
        self.prefixedJointNames = [s + self.jointPrefix for s in TrajectoryFollower.jointNames]
        self.jointStatePublisher = jointStatePublisher
        self.timestep = int(robot.getBasicTimeStep())
        self.motors = []
        self.sensors = []
        for name in TrajectoryFollower.jointNames:
            self.motors.append(robot.getMotor(name))
            self.sensors.append(robot.getPositionSensor(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
        self.goal_handle = None
        self.last_point_sent = True
        self.trajectory = None
        self.joint_goal_tolerances = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.server = ActionServer(self.node, FollowJointTrajectory, "follow_joint_trajectory",
                                   execute_callback=self.update,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel)

    def init_trajectory(self):
        """Initialize a new target trajectory."""
        state = self.jointStatePublisher.last_joint_states
        self.trajectory_t0 = self.robot.getTime()
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = self.prefixedJointNames
        self.trajectory.points = [JointTrajectoryPoint(
            positions=state.position if state else [0] * 6,
            velocities=[0] * 6,
            accelerations=[0] * 6,
            time_from_start=Duration())]

    def start(self):
        """Initialize and start the action server."""
        self.init_trajectory()
        #self.server.start() TODO
        print("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        self.node.get_logger().info(goal_handle.__class__.__name__)
        # Checks if the joints are just incorrect
        if set(goal_handle.trajectory.joint_names) != set(self.prefixedJointNames):
            #goal_handle.set_rejected()
            self.node.get_logger().info("Received a goal with incorrect joint names: (%s)" % ', '.join(goal_handle.trajectory.joint_names))
            return GoalResponse.REJECT

        if not trajectory_is_finite(goal_handle.trajectory):
            self.node.get_logger().info("Received a goal with infinites or NaNs")
            #goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
            return GoalResponse.REJECT

        # Checks that the trajectory has velocities
        if not has_velocities(goal_handle.trajectory):
            self.node.get_logger().info("Received a goal without velocities")
            #goal_handle.set_rejected(text="Received a goal without velocities")
            return GoalResponse.REJECT

        # Orders the joints of the trajectory according to joint_names
        reorder_trajectory_joints(goal_handle.trajectory, self.prefixedJointNames)

        # Inserts the current setpoint at the head of the trajectory
        now = self.robot.getTime()
        point0 = sample_trajectory(self.trajectory, now - self.trajectory_t0)
        point0.time_from_start = Duration()
        goal_handle.trajectory.points.insert(0, point0)
        self.trajectory_t0 = now

        # Replaces the goal
        self.goal_handle = goal_handle
        self.trajectory = goal_handle.trajectory
        #goal_handle.accepted()
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        if goal_handle == self.goal_handle:
            # stop the motors
            for i in range(len(TrajectoryFollower.jointNames)):
                self.motors[i].setPosition(self.sensors[i].getValue())
            #self.goal_handle.set_canceled()
            self.goal_handle = None
        else:
            pass #goal_handle.set_canceled()
        return CancelResponse.ACCEPT

    def update(self, goal_handle):
        self.node.get_logger().info('aaa')
        self.node.get_logger().info(goal_handle.__class__.__name__)
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        self.node.get_logger().info(result.__class__.__name__)
        if self.robot and self.trajectory:
            now = self.robot.getTime()
            if (now - self.trajectory_t0) <= (self.trajectory.points[-1].time_from_start.sec + self.trajectory.points[-1].time_from_start.nanosec * 1.0e-6):  # Sending intermediate points
                self.last_point_sent = False
                setpoint = sample_trajectory(self.trajectory, now - self.trajectory_t0)
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    # Velocity control is not used on the real robot and gives bad results in the simulation
                    # self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
            elif not self.last_point_sent:  # All intermediate points sent, sending last point to make sure we reach the goal.
                last_point = self.trajectory.points[-1]
                state = self.jointStatePublisher.last_joint_states
                position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                setpoint = sample_trajectory(self.trajectory, self.trajectory.points[-1].time_from_start.sec + self.trajectory.points[-1].time_from_start.nanosec * 1.0e-6)
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    # Velocity control is not used on the real robot and gives bad results in the simulation
                    # self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
            else:  # Off the end
                if self.goal_handle:
                    last_point = self.trajectory.points[-1]
                    state = self.jointStatePublisher.last_joint_states
                    position_in_tol = within_tolerance(state.position, last_point.positions, [0.1] * 6)
                    velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05] * 6)
                    if position_in_tol and velocity_in_tol:
                        # The arm reached the goal (and isn't moving) => Succeeded
                        #self.goal_handle.succeed()
                        self.goal_handle = None
        return result


class ActionServerNode(Node):

    def __init__(self):
        super().__init__('ur_driver')
        self.robot = Robot()
        self.jointStatePublisher = JointStatePublisher(self.robot, 'TODO', self)
        self.trajectoryFollower = TrajectoryFollower(self.robot, self, self.jointStatePublisher, jointPrefix='TODO')
        self.trajectoryFollower.start()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.clockPublisher = self.create_publisher(Clock, 'topic', 10)
        timer_period = 0.001 * self.timestep  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        self.get_logger().info('Time: "%lf" %lf' % (self.robot.getTime(), self.timestep))
        # Publish clock
        msg = Clock()
        time = self.robot.getTime()
        msg.clock.sec = int(time)
        # round prevents precision issues that can cause problems with ROS timers
        msg.clock.nanosec = int(round(1000 * (time - msg.clock.sec)) * 1.0e+6)
        self.clockPublisher.publish(msg)
        # update joint state and trajectory follower
        self.jointStatePublisher.publish()
        #self.trajectoryFollower.update()
        # Robot step
        i = self.robot.step(self.timestep)
        self.get_logger().info('%d' % i)
        if self.robot.step(self.timestep) < 0.0:
            del self.robot
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    actionServer = ActionServerNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(actionServer) #, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actionServer.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
