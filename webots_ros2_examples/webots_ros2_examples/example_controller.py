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

from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_msgs.srv import SetDifferentialWheelSpeed

import rclpy

from std_msgs.msg import Float64


class ExampleController(WebotsNode):

    def __init__(self, args):
        super().__init__('example_controller', args)
        self.sensorTimer = self.create_timer(0.001 * self.timestep, self.sensor_callback)
        self.leftMotor = self.robot.getMotor('motor.left')
        self.rightMotor = self.robot.getMotor('motor.right')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.motorService = self.create_service(SetDifferentialWheelSpeed, 'motor',
                                                self.motor_callback)
        self.sensorPublisher = self.create_publisher(Float64, 'sensor', 10)
        # front central proximity sensor
        self.frontSensor = self.robot.getDistanceSensor('prox.horizontal.2')
        self.frontSensor.enable(self.timestep)

    def sensor_callback(self):
        # Publish distance sensor value
        msg = Float64()
        msg.data = self.frontSensor.getValue()
        self.sensorPublisher.publish(msg)

    def motor_callback(self, request, response):
        self.leftMotor.setVelocity(request.left_speed)
        self.rightMotor.setVelocity(request.right_speed)
        return response


def main(args=None):
    rclpy.init(args=args)

    exampleController = ExampleController(args=args)

    rclpy.spin(exampleController)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
