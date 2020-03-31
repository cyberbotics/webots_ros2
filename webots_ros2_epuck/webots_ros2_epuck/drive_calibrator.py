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
#
# This node helps you to calibrate wheel radius and distance between the wheels
# by moving the robot forward and correcting wheel radius, and rotating robot and
# correcting distance between the wheels.
# ros2 run webots_ros2_epuck drive_calibrator --ros-args -p type:=linear -p wheel_radius:=0.021

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg._parameter import Parameter
from rclpy.parameter import ParameterType, ParameterValue
from geometry_msgs.msg import Twist
from webots_ros2_core.math_utils import quaternion_to_euler


# Target distance for robot to pass in meters
DEFAULT_DISTANCE = 0.1335
# Default separation between two wheels (from e-puck website)
DEFAULT_WHEEL_DISTANCE = 0.0552
# Default wheel radius (from e-puck website)
DEFAULT_WHEEL_RADIUS = 0.021


class EPuckDriveCalibrator(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

        self.test_done = False
        self.rotation_count = 0
        self.last_yaw = 0

        # Parameters
        self.type = self.declare_parameter('type', 'rotation')
        self.distance = self.declare_parameter('distance', DEFAULT_DISTANCE)
        self.wheel_distance = self.declare_parameter(
            'wheel_distance', DEFAULT_WHEEL_DISTANCE)
        self.wheel_radius = self.declare_parameter(
            'wheel_radius', DEFAULT_WHEEL_RADIUS)

        # Topics
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameter service
        self.cli = self.create_client(SetParameters, 'epuck/set_parameters')
        self.cli.wait_for_service(timeout_sec=1.0)
        self.set_param('wheel_distance', self.wheel_distance.value)
        self.set_param('wheel_radius', self.wheel_radius.value)
        self.get_logger().info('Setting wheel distance to: {}m'.format(self.wheel_distance.value))
        self.get_logger().info('Setting wheel radius to: {}m'.format(self.wheel_radius.value))

    def set_param(self, name, value):
        req = SetParameters.Request()
        param_value = ParameterValue(
            double_value=value, type=ParameterType.PARAMETER_DOUBLE)
        param = Parameter(name=name, value=param_value)
        req.parameters.append(param)
        self.cli.call_async(req)

    def send_stop(self):
        self.test_done = True
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def odometry_callback(self, msg: Odometry):
        if not self.test_done and self.type.value == 'rotation':
            # Send velocity
            self.get_logger().info('Rotation calibration in progress...')
            msg_twist = Twist()
            msg_twist.angular.x = 0.0
            msg_twist.angular.y = 0.0
            msg_twist.angular.z = 1.0
            msg_twist.linear.x = 0.0
            msg_twist.linear.y = 0.0
            msg_twist.linear.z = 0.0
            self.pub.publish(msg_twist)

            # Receive data
            yaw, _, _ = quaternion_to_euler(msg.pose.pose.orientation)
            if yaw > 0 and self.last_yaw < 0:
                self.rotation_count += 1
            if self.rotation_count == 4:
                self.send_stop()
            self.last_yaw = yaw

            self.get_logger().info('Circle: {}; Current angle: {}'.format(
                self.rotation_count, yaw))

        if not self.test_done and self.type.value == 'linear':
            # Send velocity
            self.get_logger().info('Rotation calibration in progress...')
            msg_twist = Twist()
            msg_twist.angular.x = 0.0
            msg_twist.angular.y = 0.0
            msg_twist.angular.z = 0.0
            msg_twist.linear.x = 0.02
            msg_twist.linear.y = 0.0
            msg_twist.linear.z = 0.0
            self.pub.publish(msg_twist)

            # Receive data
            print('X', msg.pose.pose.position.x)
            if msg.pose.pose.position.x > self.distance.value:
                self.send_stop()


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckDriveCalibrator(
        'epuck_drive_calibrator', args=args)
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
