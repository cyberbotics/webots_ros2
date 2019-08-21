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

"""ROS2 example controller."""

import os
import sys

from time import sleep

from webots_ros2_core.utils import get_webots_version

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from example_interfaces.srv import AddTwoInts
from rosgraph_msgs.msg import Clock

try:
    sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'python%d%d' %
                    (sys.version_info[0], sys.version_info[1])))
    from controller import Robot
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class ExampleController(Node):

    def __init__(self):
        super().__init__('example_controller')
        if get_webots_version() == 'R2019b':
            sleep(10)  # TODO: wait to make sure that Webots is started
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.clockPublisher = self.create_publisher(Clock, 'topic', 10)
        timer_period = 0.001 * self.timestep  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.leftMotor = self.robot.getMotor('motor.left')
        self.rightMotor = self.robot.getMotor('motor.right')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.motorService = self.create_service(AddTwoInts, 'motor',
                                                self.motor_callback)
        self.sensorPublisher = self.create_publisher(Float64, 'sensor', 10)
        # front central proximity sensor
        self.frontSensor = self.robot.getDistanceSensor('prox.horizontal.2')
        self.frontSensor.enable(self.timestep)

    def timer_callback(self):
        # Publish clock
        msg = Clock()
        time = self.robot.getTime()
        msg.clock.sec = int(time)
        # round prevents precision issues causing problems with ROS timers
        msg.clock.nanosec = int(round(1000 * (time - msg.clock.sec)) * 1.0e+6)
        self.clockPublisher.publish(msg)
        # self.get_logger().info('Time: "%lf"' % self.robot.getTime())
        # Publish distance sensor value
        msg = Float64()
        msg.data = self.frontSensor.getValue()
        self.sensorPublisher.publish(msg)
        # Robot step
        if self.robot.step(self.timestep) < 0.0:
            self.destroy_timer(self.timer)
            self.destroy_node()
            sys.exit(0)

    def motor_callback(self, request, response):
        self.leftMotor.setVelocity(request.a)
        self.rightMotor.setVelocity(request.b)
        return response


def main(args=None):
    rclpy.init(args=args)

    exampleController = ExampleController()

    rclpy.spin(exampleController)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
