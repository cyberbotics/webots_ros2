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

"""ROS2 TIAGo controller."""

from webots_ros2_core.webots_node import WebotsNode

import rclpy

from geometry_msgs.msg import Twist


class TiagoController(WebotsNode):

    def __init__(self, args):
        super().__init__('tiago_controller', args)
        self.leftMotor = self.robot.getMotor('wheel_left_joint')
        self.rightMotor = self.robot.getMotor('wheel_right_joint')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.motorMaxSpeed = self.leftMotor.getMaxVelocity()
        self.cmdVelSubscriber = self.create_subscription(Twist, 'cmd_vel',
                                                         self.cmdVel_callback,
                                                         10)

    def cmdVel_callback(self, msg):
        wheelGap = 0.404  # in meter
        wheelRadius = 0.1955  # in meter
        leftSpeed = ((2.0 * msg.linear.x - msg.angular.z * wheelGap) /
                     (2.0 * wheelRadius))
        rightSpeed = ((2.0 * msg.linear.x + msg.angular.z * wheelGap) /
                      (2.0 * wheelRadius))
        leftSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed,
                                                leftSpeed))
        rightSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed,
                                                 rightSpeed))
        self.leftMotor.setVelocity(leftSpeed)
        self.rightMotor.setVelocity(rightSpeed)


def main(args=None):
    rclpy.init(args=args)

    tiagoController = TiagoController(args=args)

    rclpy.spin(tiagoController)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
