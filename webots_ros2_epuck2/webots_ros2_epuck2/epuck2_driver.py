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

"""ROS2 EPuck2 controller."""

from webots_ros2_core.webots_node import WebotsNode
import rclpy
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist


WHEEL_DISTANCE = 0.052
WHEEL_RADIUS = 0.0205
CAMERA_PERIOD_MS = 500


class EPuck2Controller(WebotsNode):
    def __init__(self, args):
        super().__init__('epuck2_controller', args)

        # Initialize motors
        self.left_motor = self.robot.getMotor('left wheel motor')
        self.right_motor = self.robot.getMotor('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Intialize distance sensors
        self.sensor_publishers = []
        self.sensors = []
        for i in range(8):
            sensor = self.robot.getDistanceSensor('ps{}'.format(i))
            sensor.enable(self.timestep)
            sensor_publisher = self.create_publisher(
                Range, '/distance/ps{}'.format(i), 10)
            self.sensors.append(sensor)
            self.sensor_publishers.append(sensor_publisher)
        self.create_timer(0.01 * self.timestep, self.distance_callback)

        # Initialize camera
        self.camera = self.robot.getCamera('camera')
        self.camera.enable(CAMERA_PERIOD_MS)
        self.camera_publisher = self.create_publisher(Image, '/rgb/image', 10)
        self.create_timer(0.01 * self.timestep, self.camera_callback)

    def camera_callback(self):
        msg = Image()
        msg.height = self.camera.getHeight()
        msg.width = self.camera.getWidth()
        msg.is_bigendian = False
        msg.step = self.camera.getWidth() * 4
        msg.data = self.camera.getImage()
        msg.encoding = 'bgra8'
        self.camera_publisher.publish(msg)

    def cmd_vel_callback(self, twist):
        left_velocity = (2.0 * twist.linear.x - twist.angular.z *
                         WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS)
        right_velocity = (2.0 * twist.linear.x + twist.angular.z *
                          WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def distance_callback(self):
        for i in range(8):
            msg = Range()
            msg.field_of_view = self.sensors[i].getAperture()
            msg.min_range = self.sensors[i].getMinValue()
            msg.max_range = self.sensors[i].getMaxValue()
            msg.range = self.sensors[i].getValue()
            msg.radiation_type = Range.INFRARED
            self.sensor_publishers[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    epuck2_controller = EPuck2Controller(args=args)

    rclpy.spin(epuck2_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
