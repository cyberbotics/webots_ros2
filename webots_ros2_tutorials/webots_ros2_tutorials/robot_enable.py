# Copyright 1996-2020 Soft_illusion.
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
# This file is used to enable all the sensors and making robot move

import rclpy
from webots_ros2_core.webots_node import WebotsNode
from geometry_msgs.msg import Twist

DEVICE_CONFIG = {
    'camera_mid': {'topic_name': 'camera', 'timestep': 16}}


class RobotEnable(WebotsNode):
    def __init__(self, args):
        super().__init__('sensor_enable', args)

        # Front wheels
        self.left_motor_front = self.robot.getMotor('left_front_wheel')
        self.left_motor_front.setPosition(float('inf'))
        self.left_motor_front.setVelocity(0)

        self.right_motor_front = self.robot.getMotor('right_front_wheel')
        self.right_motor_front.setPosition(float('inf'))
        self.right_motor_front.setVelocity(0)

        # Rear wheels
        self.left_motor_rear = self.robot.getMotor('left_rear_wheel')
        self.left_motor_rear.setPosition(float('inf'))
        self.left_motor_rear.setVelocity(0)

        self.right_motor_rear = self.robot.getMotor('right_rear_wheel')
        self.right_motor_rear.setPosition(float('inf'))
        self.right_motor_rear.setVelocity(0)

        self.motor_max_speed = self.left_motor_rear.getMaxVelocity()

        # Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_velocity_callback, 1)

        self.start_device_manager(DEVICE_CONFIG)
        self.get_logger().info('Sensor enabled')

    def cmd_velocity_callback(self, msg):
        wheel_gap = 0.1  # in meter
        wheel_radius = 0.04  # in meter

        left_speed = ((2.0 * msg.linear.x - msg.angular.z *
                       wheel_gap) / (2.0 * wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z *
                        wheel_gap) / (2.0 * wheel_radius))
        left_speed = min(self.motor_max_speed,
                         max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed,
                          max(-self.motor_max_speed, right_speed))

        self.left_motor_front.setVelocity(left_speed)
        self.right_motor_front.setVelocity(right_speed)
        self.left_motor_rear.setVelocity(left_speed)
        self.right_motor_rear.setVelocity(right_speed)


def main(args=None):
    rclpy.init(args=args)
    robot_object = RobotEnable(args=args)
    rclpy.spin(robot_object)

    robot_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
