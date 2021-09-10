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

import sys

from math import cos, sin
import rclpy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.utils import get_node_name_from_args


class WebotsDifferentialDriveNode(WebotsNode):
    """
    Extends WebotsNode to allow easy integration with differential drive robots.

    Args:
    ----
        name (WebotsNode): Webots Robot node.
        args (dict): Arguments passed to ROS2 base node.
        wheel_distance (float): Distance between two wheels (axle length) in meters.
        wheel_radius (float): Radius of both wheels in meters.
        left_joint (str): Name of motor associated with left wheel.
        right_joint (str): Name of motor associated with right wheel.
        left_encoder (str): Name of encoder associated with left wheel.
        right_encoder (str): Name of encoder associated with right wheel.
        command_topic (str): Name of topic to which
            [`geometry_msgs/Twist`](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg)
            the node is subscribed to.
        odometry_topic (str): Name of topic to which
            [`nav_msgs/Odometry`](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg)
            messages are published.
        odometry_frame (str): Name of odometry frame.
        robot_base_frame (str): Name of robot's base link.

    """

    def __init__(self,
                 name,
                 args,
                 wheel_distance=0,
                 wheel_radius=0,
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

        # Parametrise
        wheel_distance_param = self.declare_parameter('wheel_distance', wheel_distance)
        wheel_radius_param = self.declare_parameter('wheel_radius', wheel_radius)
        left_joint_param = self.declare_parameter('left_joint', left_joint)
        right_joint_param = self.declare_parameter('right_joint', right_joint)
        left_encoder_param = self.declare_parameter('left_encoder', left_encoder)
        right_encoder_param = self.declare_parameter('right_encoder', right_encoder)
        command_topic_param = self.declare_parameter('command_topic', command_topic)
        odometry_topic_param = self.declare_parameter('odometry_topic', odometry_topic)
        odometry_frame_param = self.declare_parameter('odometry_frame', odometry_frame)
        robot_base_frame_param = self.declare_parameter('robot_base_frame', robot_base_frame)
        self._wheel_radius = wheel_radius_param.value
        self._wheel_distance = wheel_distance_param.value
        self.set_parameters_callback(self._on_param_changed)
        if self._wheel_radius == 0 or self._wheel_distance == 0:
            self.get_logger().error('Parameters `wheel_distance` and `wheel_radius` have to greater than 0')
            self.destroy_node()
            sys.exit(1)
        self.get_logger().info(
            f'Initializing differential drive node with wheel_distance = {self._wheel_distance} and ' +
            f'wheel_radius = {self._wheel_radius}'
        )

        # Store config
        self._odometry_frame = odometry_frame_param.value
        self._robot_base_frame = robot_base_frame_param.value

        # Initialize motors
        self.left_motor = self.robot.getMotor(left_joint_param.value)
        self.right_motor = self.robot.getMotor(right_joint_param.value)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.create_subscription(Twist, command_topic_param.value, self._cmd_vel_callback, 1)

        # Initialize odometry
        self.reset_odometry()
        self.left_wheel_sensor = self.robot.getPositionSensor(left_encoder_param.value)
        self.right_wheel_sensor = self.robot.getPositionSensor(right_encoder_param.value)
        self.left_wheel_sensor.enable(self.timestep)
        self.right_wheel_sensor.enable(self.timestep)
        self._odometry_publisher = self.create_publisher(Odometry, odometry_topic_param.value, 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize timer
        self._last_odometry_sample_time = self.robot.getTime()

    def _cmd_vel_callback(self, twist):
        self.get_logger().info('Message received')
        right_velocity = twist.linear.x + self._wheel_distance * twist.angular.z / 2
        left_velocity = twist.linear.x - self._wheel_distance * twist.angular.z / 2
        left_omega = left_velocity / (self._wheel_radius)
        right_omega = right_velocity / (self._wheel_radius)
        self.left_motor.setVelocity(left_omega)
        self.right_motor.setVelocity(right_omega)

    def step(self, ms):
        super().step(ms)

        stamp = Time(seconds=self.robot.getTime()).to_msg()

        time_diff_s = self.robot.getTime() - self._last_odometry_sample_time
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()

        if time_diff_s == 0.0:
            return

        # Calculate velocities
        v_left_rad = (left_wheel_ticks - self._prev_left_wheel_ticks) / time_diff_s
        v_right_rad = (right_wheel_ticks - self._prev_right_wheel_ticks) / time_diff_s
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
        k10 = v * cos(self._prev_angle + time_diff_s * k02 / 2)
        k11 = v * sin(self._prev_angle + time_diff_s * k02 / 2)
        k12 = omega
        k20 = v * cos(self._prev_angle + time_diff_s * k12 / 2)
        k21 = v * sin(self._prev_angle + time_diff_s * k12 / 2)
        k22 = omega
        k30 = v * cos(self._prev_angle + time_diff_s * k22 / 2)
        k31 = v * sin(self._prev_angle + time_diff_s * k22 / 2)
        k32 = omega
        position = [
            self._prev_position[0] + (time_diff_s / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self._prev_position[1] + (time_diff_s / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self._prev_angle + \
            (time_diff_s / 6) * (k02 + 2 * (k12 + k22) + k32)

        # Update variables
        self._prev_position = position.copy()
        self._prev_angle = angle
        self._prev_left_wheel_ticks = left_wheel_ticks
        self._prev_right_wheel_ticks = right_wheel_ticks
        self._last_odometry_sample_time = self.robot.getTime()

        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odometry_frame
        msg.child_frame_id = self._robot_base_frame
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.orientation.z = sin(angle / 2)
        msg.pose.pose.orientation.w = cos(angle / 2)
        self._odometry_publisher.publish(msg)

        # Pack & publish transforms
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odometry_frame
        tf.child_frame_id = self._robot_base_frame
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = sin(angle / 2)
        tf.transform.rotation.w = cos(angle / 2)
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


def main(args=None):
    rclpy.init(args=args)

    webots_robot_name = get_node_name_from_args()
    differential_drive = WebotsDifferentialDriveNode(webots_robot_name, args=args)
    differential_drive.start_device_manager()
    rclpy.spin(differential_drive)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
