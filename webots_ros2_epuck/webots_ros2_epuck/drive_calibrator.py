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
#
# This node helps you to calibrate wheel radius and distance between the wheels
# by moving the robot forward and correcting wheel radius, and rotating robot and
# correcting distance between the wheels.

import sys
import time
from math import pi, atan2
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import Twist


DEFAULT_DISTANCE = 0.1335
NUMBER_OF_ROTATIONS = 4
LINEAR_VELOCITY = 0.02
ANGULAR_VELOCITY = 1


class EPuckDriveCalibrator(Node):
    def __init__(self, name):
        super().__init__(name)

        # Parameters
        self.type = self.declare_parameter('type', 'rotation')
        self.distance = self.declare_parameter('distance', DEFAULT_DISTANCE)

        # Topics
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Odometry
        self.odom_angular_last = 0.0
        self.odom_angular_last_abs = 0.0
        self.odom_angular_start = 0.0
        self.odom_linear_start = 0.0
        self.odom_params_initialised = False

    def finish_calibration(self):
        self.set_velocity(0, 0)
        self.get_logger().info('The robot has reached the given pose according to odometry')
        time.sleep(0.5)
        self.destroy_node()
        sys.exit(0)

    def set_velocity(self, linear, angular):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def odometry_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.odom_params_initialised:
            if yaw < 0:
                yaw = 2 * pi + yaw
            self.odom_params_initialised = True
            self.odom_angular_last = yaw
            self.odom_angular_start = yaw
            self.odom_angular_last_abs = yaw
            self.odom_linear_start = msg.pose.pose.position.x
            return

        # Resolve singularities (first for positive angle and then for negative angle)
        if yaw - self.odom_angular_last > pi:
            self.odom_angular_last_abs += 2*pi - (yaw - self.odom_angular_last)
        elif self.odom_angular_last - yaw > pi:
            self.odom_angular_last_abs += 2*pi - (self.odom_angular_last - yaw)
        else:
            self.odom_angular_last_abs += yaw - self.odom_angular_last

        # Angular calibration
        if self.type.value == 'angular':
            self.get_logger().info('Angular calibration in progress...')
            self.set_velocity(0, ANGULAR_VELOCITY)
            n_rotations = (self.odom_angular_last_abs - self.odom_angular_start)/(2*pi)
            if n_rotations > NUMBER_OF_ROTATIONS:
                self.finish_calibration()
            self.get_logger().info(f'Number of rotations: {n_rotations:.4f}')

        # Linear calibration
        if self.type.value == 'linear':
            self.get_logger().info('Linear calibration in progress...')
            self.set_velocity(LINEAR_VELOCITY, 0)
            passed_distance = msg.pose.pose.position.x - self.odom_linear_start
            self.get_logger().info(f'Passed distance: {passed_distance:.4f}')
            if passed_distance > self.distance.value:
                self.finish_calibration()

        # Save readings
        self.odom_angular_last = yaw


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckDriveCalibrator('epuck_drive_calibrator')
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
