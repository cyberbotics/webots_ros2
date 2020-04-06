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

from math import cos, sin
from webots_ros2_core.webots_node import WebotsNode
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from webots_ros2_core.math_utils import euler_to_quaternion
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


class WebotsDifferentialDriveNode(WebotsNode):
    def __init__(self,
                 name,
                 args,
                 wheel_distance,
                 wheel_radius,
                 left_joint='left wheel motor',
                 right_joint='right wheel motor',
                 left_encoder='left wheel sensor',
                 right_encoder='right wheel sensor',
                 command_topic='/cmd_vel',
                 odometry_topic='/odom',
                 odometry_frame='odom',
                 robot_base_frame='base_link'
                 ):
        super().__init__(name, args)

        # Store config
        self._odometry_frame = odometry_frame
        self._robot_base_frame = robot_base_frame

        # Parametrise
        wheel_distance_param = self.declare_parameter(
            "wheel_distance", wheel_distance)
        wheel_radius_param = self.declare_parameter(
            "wheel_radius", wheel_radius)
        self._wheel_radius = wheel_radius_param.value
        self._wheel_distance = wheel_distance_param.value
        self.set_parameters_callback(self._on_param_changed)

        # Initialize motors
        self.left_motor = self.robot.getMotor(left_joint)
        self.right_motor = self.robot.getMotor(right_joint)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.create_subscription(Twist, command_topic,
                                 self._cmd_vel_callback, 1)

        # Initialize odometry
        self.reset_odometry()
        self.left_wheel_sensor = self.robot.getPositionSensor(left_encoder)
        self.right_wheel_sensor = self.robot.getPositionSensor(right_encoder)
        self.left_wheel_sensor.enable(self.timestep)
        self.right_wheel_sensor.enable(self.timestep)
        self._odometry_publisher = self.create_publisher(
            Odometry, odometry_topic, 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize timer
        self.create_timer(self.timestep / 1000.0, self._publish_odometry_data)

    def _cmd_vel_callback(self, twist):
        self.get_logger().info('Message received')
        left_velocity = (2.0 * twist.linear.x - twist.angular.z *
                         self._wheel_distance) / (2.0 * self._wheel_radius)
        right_velocity = (2.0 * twist.linear.x + twist.angular.z *
                          self._wheel_distance) / (2.0 * self._wheel_radius)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def _publish_odometry_data(self):
        stamp = self.now()

        encoder_period_s = self.timestep / 1000.0
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()

        # Calculate velocities
        v_left_rad = (left_wheel_ticks -
                      self._prev_left_wheel_ticks) / encoder_period_s
        v_right_rad = (right_wheel_ticks -
                       self._prev_right_wheel_ticks) / encoder_period_s
        v_left = v_left_rad * self._wheel_radius
        v_right = v_right_rad * self._wheel_radius
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self._wheel_distance

        # Calculate position & angle
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(self._prev_angle)
        k01 = v * sin(self._prev_angle)
        k02 = omega
        k10 = v * cos(self._prev_angle + encoder_period_s * k02 / 2)
        k11 = v * sin(self._prev_angle + encoder_period_s * k02 / 2)
        k12 = omega
        k20 = v * cos(self._prev_angle + encoder_period_s * k12 / 2)
        k21 = v * sin(self._prev_angle + encoder_period_s * k12 / 2)
        k22 = omega
        k30 = v * cos(self._prev_angle + encoder_period_s * k22 / 2)
        k31 = v * sin(self._prev_angle + encoder_period_s * k22 / 2)
        k32 = omega
        position = [
            self._prev_position[0] + (encoder_period_s / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self._prev_position[1] + (encoder_period_s / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self._prev_angle + \
            (encoder_period_s / 6) * (k02 + 2 * (k12 + k22) + k32)

        # Update variables
        self._prev_position = position.copy()
        self._prev_angle = angle
        self._prev_left_wheel_ticks = left_wheel_ticks
        self._prev_right_wheel_ticks = right_wheel_ticks

        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odometry_frame
        msg.child_frame_id = self._robot_base_frame
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.orientation = euler_to_quaternion(0, 0, angle)
        self._odometry_publisher.publish(msg)

        # Pack & publish transforms
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odometry_frame
        tf.child_frame_id = self._robot_base_frame
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation = euler_to_quaternion(0, 0, angle)
        self._tf_broadcaster.sendTransform(tf)

    def _on_param_changed(self, params):
        result = SetParametersResult()
        result.successful = True

        for param in params:
            if param.name == "wheel_radius":
                self.reset_odometry()
                self._wheel_radius = param.value
            elif param.name == "wheel_distance":
                self.reset_odometry()
                self._wheel_distance = param.value

        return result

    def reset_odometry(self):
        self._prev_left_wheel_ticks = 0
        self._prev_right_wheel_ticks = 0
        self._prev_position = (0.0, 0.0)
        self._prev_angle = 0.0
