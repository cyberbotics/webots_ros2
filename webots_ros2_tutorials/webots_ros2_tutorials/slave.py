# Copyright 1996-2021 Soft_illusion.
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
        self.sensor_timer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback)

        self.right_sensor = self.robot.getDistanceSensor(
            'distance_sensor_right')
        self.right_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_right = self.create_publisher(
            Float64, 'right_IR', 1)

        self.mid_sensor = self.robot.getDistanceSensor('distance_sensor_mid')
        self.mid_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_mid = self.create_publisher(Float64, 'mid_IR', 1)

        self.left_sensor = self.robot.getDistanceSensor('distance_sensor_left')
        self.left_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_left = self.create_publisher(
            Float64, 'left_IR', 1)

        self.get_logger().info('Sensor enabled')

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
            Twist, 'cmd_vel', self.cmdVel_callback, 1)

    def cmdVel_callback(self, msg):
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

    def sensor_callback(self):
        # Publish distance sensor value
        msg_right = Float64()
        msg_right.data = self.right_sensor.getValue()
        self.sensor_publisher_right.publish(msg_right)

        msg_mid = Float64()
        msg_mid.data = self.mid_sensor.getValue()
        self.sensor_publisher_mid.publish(msg_mid)

        msg_left = Float64()
        msg_left.data = self.left_sensor.getValue()
        self.sensor_publisher_left.publish(msg_left)


def main(args=None):
    rclpy.init(args=args)
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
