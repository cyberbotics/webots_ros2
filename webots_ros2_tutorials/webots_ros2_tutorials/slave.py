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

import rclpy
from webots_ros2_core.webots_node import WebotsNode
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class service_node_vel(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_node', args)
        # Sensor section
        self.sensorTimer = self.create_timer(0.001 * self.timestep,
                                             self.sensor_callback)

        # Enable 3 sensors
        self.service_node_vel_timestep = 16

        self.right_sensor = self.robot.getDistanceSensor('ls_right')
        self.right_sensor.enable(self.timestep)
        self.sensorPublisher_right = self.create_publisher(Float64, 'right_IR', 1)

        self.mid_sensor = self.robot.getDistanceSensor('ls_mid')
        self.mid_sensor.enable(self.timestep)
        self.sensorPublisher_mid = self.create_publisher(Float64, 'mid_IR', 1)

        self.left_sensor = self.robot.getDistanceSensor('ls_left')
        self.left_sensor.enable(self.timestep)
        self.sensorPublisher_left = self.create_publisher(Float64, 'left_IR', 1)

        self.get_logger().info('Sensor enabled')

        # Wheels section
        # |1 2|
        # |3 4|
        # Front wheels
        self.leftMotor_front = self.robot.getMotor('wheel1')
        self.leftMotor_front.setPosition(float('inf'))
        self.leftMotor_front.setVelocity(0)

        self.rightMotor_front = self.robot.getMotor('wheel2')
        self.rightMotor_front.setPosition(float('inf'))
        self.rightMotor_front.setVelocity(0)

        # Rear wheels
        self.leftMotor_rear = self.robot.getMotor('wheel3')
        self.leftMotor_rear.setPosition(float('inf'))
        # self.leftMotor_rear.setVelocity(0)

        self.rightMotor_rear = self.robot.getMotor('wheel4')
        self.rightMotor_rear.setPosition(float('inf'))
        # self.rightMotor_rear.setVelocity(0)

        self.motorMaxSpeed = self.leftMotor_rear.getMaxVelocity()

        # Create Subscriber
        self.cmdVelSubscriber = self.create_subscription(Twist, 'cmd_vel', self.cmdVel_callback, 1)

    def cmdVel_callback(self, msg):
        wheelGap = 0.1  # in meter
        wheelRadius = 0.04  # in meter

        leftSpeed = ((2.0 * msg.linear.x - msg.angular.z * wheelGap) / (2.0 * wheelRadius))
        rightSpeed = ((2.0 * msg.linear.x + msg.angular.z * wheelGap) / (2.0 * wheelRadius))
        leftSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed, leftSpeed))
        rightSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed, rightSpeed))

        self.leftMotor_front.setVelocity(leftSpeed)
        self.rightMotor_front.setVelocity(rightSpeed)
        self.leftMotor_rear.setVelocity(leftSpeed)
        self.rightMotor_rear.setVelocity(rightSpeed)

    def sensor_callback(self):
        # Publish distance sensor value
        msg_right = Float64()
        msg_right.data = self.right_sensor.getValue()
        self.sensorPublisher_right.publish(msg_right)

        msg_mid = Float64()
        msg_mid.data = self.mid_sensor.getValue()
        self.sensorPublisher_mid.publish(msg_mid)

        msg_left = Float64()
        msg_left.data = self.left_sensor.getValue()
        self.sensorPublisher_left.publish(msg_left)


def main(args=None):
    rclpy.init(args=args)
    client_vel = service_node_vel(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
