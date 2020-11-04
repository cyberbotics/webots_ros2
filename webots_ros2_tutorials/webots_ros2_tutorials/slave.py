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


class ServiceNodeVelocity(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_node', args)

        # Enable 3 sensors
        self.service_node_vel_timestep = 16

        # Sensor section
        self.sensorTimer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback)

        self.right_sensor = self.robot.getDistanceSensor(
            'distance_sensor_right')
        self.right_sensor.enable(self.service_node_vel_timestep)
        self.sensorPublisher_right = self.create_publisher(
            Float64, 'right_IR', 1)

        self.mid_sensor = self.robot.getDistanceSensor('distance_sensor_mid')
        self.mid_sensor.enable(self.service_node_vel_timestep)
        self.sensorPublisher_mid = self.create_publisher(Float64, 'mid_IR', 1)

        self.left_sensor = self.robot.getDistanceSensor('distance_sensor_left')
        self.left_sensor.enable(self.service_node_vel_timestep)
        self.sensorPublisher_left = self.create_publisher(
            Float64, 'left_IR', 1)

        self.get_logger().info('Sensor enabled')

        # Front wheels
        self.leftMotor_front = self.robot.getMotor('left_front_wheel')
        self.leftMotor_front.setPosition(float('inf'))
        self.leftMotor_front.setVelocity(0)

        self.rightMotor_front = self.robot.getMotor('right_front_wheel')
        self.rightMotor_front.setPosition(float('inf'))
        self.rightMotor_front.setVelocity(0)

        # Rear wheels
        self.leftMotor_rear = self.robot.getMotor('left_rear_wheel')
        self.leftMotor_rear.setPosition(float('inf'))
        self.leftMotor_rear.setVelocity(0)

        self.rightMotor_rear = self.robot.getMotor('right_rear_wheel')
        self.rightMotor_rear.setPosition(float('inf'))
        self.rightMotor_rear.setVelocity(0)

        self.motorMaxSpeed = self.leftMotor_rear.getMaxVelocity()

        # Create Subscriber
        self.cmdVelSubscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1)

    def cmdVel_callback(self, msg):
        wheelGap = 0.1  # in meter
        wheelRadius = 0.04  # in meter

        left_speed = ((2.0 * msg.linear.x - msg.angular.z *
                       wheelGap) / (2.0 * wheelRadius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z *
                        wheelGap) / (2.0 * wheelRadius))
        left_speed = min(self.motorMaxSpeed,
                         max(-self.motorMaxSpeed, left_speed))
        right_speed = min(self.motorMaxSpeed,
                          max(-self.motorMaxSpeed, right_speed))

        self.leftMotor_front.setVelocity(left_speed)
        self.rightMotor_front.setVelocity(right_speed)
        self.leftMotor_rear.setVelocity(left_speed)
        self.rightMotor_rear.setVelocity(right_speed)

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
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
