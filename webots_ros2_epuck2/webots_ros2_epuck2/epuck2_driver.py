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
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from webots_ros2_msgs.srv import SetInt
from functools import partial
from math import pi, cos, sin


WHEEL_DISTANCE = 0.052
WHEEL_RADIUS = 0.0205
CAMERA_PERIOD_MS = 500
ENCODER_PERIOD_MS = 100
DISTANCE_PERIOD_MS = 100
ENCODER_RESOLUTION = (2 * pi) / 1000


def euler_to_quaternion(roll, pitch, yaw):
    """
    Source: https://computergraphics.stackexchange.com/a/8229
    """
    q = Quaternion()
    q.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
        cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    q.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    q.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
        sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    q.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return q


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

        # Initialize odometry
        self.prev_left_wheel_ticks = 0
        self.prev_right_wheel_ticks = 0
        self.prev_position = (0, 0)
        self.prev_angle = 0
        self.left_wheel_sensor = self.robot.getPositionSensor(
            'left wheel sensor')
        self.right_wheel_sensor = self.robot.getPositionSensor(
            'right wheel sensor')
        self.left_wheel_sensor.enable(ENCODER_PERIOD_MS)
        self.right_wheel_sensor.enable(ENCODER_PERIOD_MS)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(ENCODER_PERIOD_MS / 1000, self.odometry_callback)

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

        sensor = self.robot.getDistanceSensor('tof')
        sensor.enable(self.timestep)
        sensor_publisher = self.create_publisher(Range, '/distance/tof', 10)
        self.sensors.append(sensor)
        self.sensor_publishers.append(sensor_publisher)

        self.create_timer(DISTANCE_PERIOD_MS / 1000, self.distance_callback)

        # Initialize camera
        self.camera = self.robot.getCamera('camera')
        self.camera.enable(CAMERA_PERIOD_MS)
        self.camera_publisher = self.create_publisher(
            Image, '/camera/image_raw', 10)
        self.create_timer(CAMERA_PERIOD_MS / 1000, self.camera_callback)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)

        # Initialize LEDs
        self.leds = []
        self.led_services = []
        for i in range(8):
            led = self.robot.getLED('led{}'.format(i))
            led_service = self.create_service(
                SetInt, '/set_led{}'.format(i), partial(self.led_callback, index=i))
            self.leds.append(led)
            self.led_services.append(led_service)

    def odometry_callback(self):
        # Calculate velocities
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()
        v_left_rad = (1 / 2 * pi) * ENCODER_RESOLUTION * \
            (left_wheel_ticks - self.prev_left_wheel_ticks) / \
            (ENCODER_PERIOD_MS / 1000)
        v_right_rad = (1 / 2 * pi) * ENCODER_RESOLUTION * \
            (right_wheel_ticks - self.prev_right_wheel_ticks) / \
            (ENCODER_PERIOD_MS / 1000)
        v_left = v_left_rad * WHEEL_RADIUS
        v_right = v_right_rad * WHEEL_RADIUS
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / WHEEL_DISTANCE

        # Calculate position & angle
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(omega)
        k01 = v * sin(omega)
        k02 = omega
        k10 = v * cos(omega + ENCODER_PERIOD_MS * k02 / 2)
        k11 = v * sin(omega + ENCODER_PERIOD_MS * k02 / 2)
        k12 = omega
        k20 = v * cos(omega + ENCODER_PERIOD_MS * k12 / 2)
        k21 = v * sin(omega + ENCODER_PERIOD_MS * k12 / 2)
        k22 = omega
        k30 = v * cos(omega + ENCODER_PERIOD_MS * k22 / 2)
        k31 = v * sin(omega + ENCODER_PERIOD_MS * k22 / 2)
        k32 = omega
        position = [
            self.prev_position[0] + (ENCODER_PERIOD_MS / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self.prev_position[1] + (ENCODER_PERIOD_MS / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self.prev_angle + \
            (ENCODER_PERIOD_MS / 6) * (k02 + 2 * (k12 + k22) + k32)

        # Update variables
        self.prev_position = position.copy()
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks

        # Pack & publish everything
        msg = Odometry()
        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.z = omega
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.orientation = euler_to_quaternion(0, 0, angle)
        self.odometry_publisher.publish(msg)

    def led_callback(self, req, res, index):
        self.leds[0].set(req.value)
        res.success = True
        return res

    def camera_callback(self):
        # Image data
        msg = Image()
        msg.height = self.camera.getHeight()
        msg.width = self.camera.getWidth()
        msg.is_bigendian = False
        msg.step = self.camera.getWidth() * 4
        msg.data = self.camera.getImage()
        msg.encoding = 'bgra8'
        self.camera_publisher.publish(msg)

        # CameraInfo data
        msg = CameraInfo()
        msg.height = self.camera.getHeight()
        msg.width = self.camera.getWidth()
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            self.camera.getFocalLength(), 0.0, self.camera.getWidth() / 2,
            0.0, self.camera.getFocalLength(), self.camera.getHeight() / 2,
            0.0, 0.0, 1.0
        ]
        msg.p = [
            self.camera.getFocalLength(), 0.0, self.camera.getWidth() / 2, 0.0,
            0.0, self.camera.getFocalLength(), self.camera.getHeight() / 2, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        self.camera_info_publisher.publish(msg)

    def cmd_vel_callback(self, twist):
        left_velocity = (2.0 * twist.linear.x - twist.angular.z *
                         WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS)
        right_velocity = (2.0 * twist.linear.x + twist.angular.z *
                          WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def distance_callback(self):
        for i in range(9):
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
