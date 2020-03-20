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

import rclpy
import math
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg._parameter import Parameter
from rclpy.parameter import ParameterType, ParameterValue
from geometry_msgs.msg import Twist


# ros2 run webots_ros2_epuck2 drive_calibrator --ros-args -p type:=linear -p wheel_radius:=0.021


def quaternion_to_euler(q):
    # Reference: https://computergraphics.stackexchange.com/a/8229
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


class EPuckDriveCalibrator(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

        self.test_done = False
        self.rotation_count = 0
        self.last_yaw = 0

        # Parameters
        self.type = self.declare_parameter('type', 'rotation')
        self.distance = self.declare_parameter('distance', 133.5 / 1000)
        self.wheel_distance = self.declare_parameter('wheel_distance', 0.0552)
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.021)

        # Topics
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameter service
        self.cli = self.create_client(SetParameters, 'epuck2/set_parameters')
        self.cli.wait_for_service(timeout_sec=1.0)
        self.set_param('wheel_distance', self.wheel_distance.value)
        self.set_param('wheel_radius', self.wheel_radius.value)
        print('Setting wheel distance to: {}m'.format(self.wheel_distance.value))
        print('Setting wheel radius to: {}m'.format(self.wheel_radius.value))

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
            print('Rotation calibration in progress...')
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

            print('Circle: {}; Current angle: {}'.format(
                self.rotation_count, yaw))

        if not self.test_done and self.type.value == 'linear':
            # Send velocity
            print('Rotation calibration in progress...')
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
    epuck2_controller = EPuckDriveCalibrator(
        'epuck_drive_calibrator', args=args)
    rclpy.spin(epuck2_controller)
    epuck2_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
