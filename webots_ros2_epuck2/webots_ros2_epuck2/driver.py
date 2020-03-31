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

"""ROS2 e-puck driver."""

from functools import partial
from math import pi, cos, sin
import rclpy
from std_msgs.msg import Bool, Int32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from sensor_msgs.msg import Range, Image, CameraInfo, Imu, LaserScan, Illuminance
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.math_utils import euler_to_quaternion, interpolate_table
from webots_ros2_core.utils import get_ros_stamp
from rcl_interfaces.msg import SetParametersResult


PERIOD_MS = 64
PERIOD_S = PERIOD_MS / 1000.0
OUT_OF_RANGE = 0.0
ENCODER_RESOLUTION = 1000.0
TOF_MAX_RANGE = 0.0
TOF_MIN_RANGE = 1.0
INFRARED_MAX_RANGE = 0.04
INFRARED_MIN_RANGE = 0.009
GROUND_MIN_RANGE = 0.0
GROUND_MAX_RANGE = 0.016
DEFAULT_WHEEL_RADIUS = 0.02
DEFAULT_WHEEL_DISTANCE = 0.05685
NB_LIGHT_SENSORS = 8
NB_GROUND_SENSORS = 3
NB_RGB_LEDS = 4
NB_BINARY_LEDS = 4
NB_INFRARED_SENSORS = 8
SENSOR_DIST_FROM_CENTER = 0.035
# https://ieee-dataport.org/open-access/conversion-guide-solar-irradiance-and-lux-illuminance
IRRADIANCE_TO_ILLUMINANCE = 120

TOF_TABLE = [
    [2.00, 2000.0],
    [1.70, 1780.5],
    [1.00, 1052.0],
    [0.50, 531.9],
    [0.20, 218.9],
    [0.10, 111.0],
    [0.05, 58.5],
    [0.00, 19.8]
]

DISTANCE_TABLE = [
    [0, 4095],
    [0.005, 2133.33],
    [0.01, 1465.73],
    [0.015, 601.46],
    [0.02, 383.84],
    [0.03, 234.93],
    [0.04, 158.03],
    [0.05, 120],
    [0.06, 104.09]
]

LIGHT_TABLE = [
    [1, 4095],
    [2, 0]
]

GROUND_TABLE = [
    [0, 1000],
    [0.016, 300]
]

DISTANCE_SENSOR_ANGLE = [
    -15 * pi / 180,   # ps0
    -45 * pi / 180,   # ps1
    -90 * pi / 180,   # ps2
    -150 * pi / 180,  # ps3
    150 * pi / 180,   # ps4
    90 * pi / 180,    # ps5
    45 * pi / 180,    # ps6
    15 * pi / 180,    # ps7
]


class EPuckDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('epuck_driver', args)

        # Parameters
        wheel_distance_param = self.declare_parameter("wheel_distance", 0.0552)
        wheel_radius_param = self.declare_parameter("wheel_radius", 0.021)
        camera_period_param = self.declare_parameter(
            "camera_period", self.timestep)
        self.period = self.declare_parameter("period", self.timestep)
        self.camera_period = camera_period_param.value
        self.wheel_radius = wheel_radius_param.value
        self.wheel_distance = wheel_distance_param.value
        self.set_parameters_callback(self.on_param_changed)

        # Initialize motors
        self.left_motor = self.robot.getMotor('left wheel motor')
        self.right_motor = self.robot.getMotor('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.get_logger().info('EPuck Initialized')

        # Initialize odometry
        self.reset_odometry()
        self.left_wheel_sensor = self.robot.getPositionSensor(
            'left wheel sensor')
        self.right_wheel_sensor = self.robot.getPositionSensor(
            'right wheel sensor')
        self.left_wheel_sensor.enable(self.period.value)
        self.right_wheel_sensor.enable(self.period.value)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 1)

        # Initialize IMU
        self.gyro = self.robot.getGyro('gyro')
        self.gyro.enable(self.period.value)
        self.accelerometer = self.robot.getAccelerometer('accelerometer')
        self.accelerometer.enable(self.period.value)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        # Initialize ground sensors
        self.ground_sensors = {}
        self.ground_sensor_publishers = {}
        self.ground_sensor_broadcasters = []
        for i in range(3):
            idx = 'gs{}'.format(i)
            ground_sensor = self.robot.getDistanceSensor(idx)
            if ground_sensor:
                ground_sensor.enable(self.period.value)
                self.ground_sensors[idx] = ground_sensor
                self.ground_sensor_publishers[idx] = self.create_publisher(
                    Range, '/' + idx, 1)

                ground_sensor_broadcaster = StaticTransformBroadcaster(self)
                ground_sensor_transform = TransformStamped()
                ground_sensor_transform.header.stamp = get_ros_stamp()
                ground_sensor_transform.header.frame_id = "base_link"
                ground_sensor_transform.child_frame_id = "gs" + str(i)
                ground_sensor_transform.transform.rotation = euler_to_quaternion(
                    0, pi/2, 0)
                ground_sensor_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER - 0.005
                ground_sensor_transform.transform.translation.y = 0.009 - i * 0.009
                ground_sensor_transform.transform.translation.z = 0.0
                ground_sensor_broadcaster.sendTransform(
                    ground_sensor_transform)
                self.ground_sensor_broadcasters.append(
                    ground_sensor_broadcaster)

        # Intialize distance sensors
        self.distance_sensor_publishers = {}
        self.distance_sensors = {}
        self.distance_sensor_broadcasters = []
        for i in range(8):
            sensor = self.robot.getDistanceSensor('ps{}'.format(i))
            sensor.enable(self.period.value)
            sensor_publisher = self.create_publisher(
                Range, '/ps{}'.format(i), 10)
            self.distance_sensors['ps{}'.format(i)] = sensor
            self.distance_sensor_publishers['ps{}'.format(
                i)] = sensor_publisher

            distance_sensor_broadcaster = StaticTransformBroadcaster(self)
            distance_sensor_transform = TransformStamped()
            distance_sensor_transform.header.stamp = get_ros_stamp()
            distance_sensor_transform.header.frame_id = "base_link"
            distance_sensor_transform.child_frame_id = "ps" + str(i)
            distance_sensor_transform.transform.rotation = euler_to_quaternion(
                0, 0, DISTANCE_SENSOR_ANGLE[i])
            distance_sensor_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER * \
                cos(DISTANCE_SENSOR_ANGLE[i])
            distance_sensor_transform.transform.translation.y = SENSOR_DIST_FROM_CENTER * \
                sin(DISTANCE_SENSOR_ANGLE[i])
            distance_sensor_transform.transform.translation.z = 0.0
            distance_sensor_broadcaster.sendTransform(
                distance_sensor_transform)
            self.distance_sensor_broadcasters.append(
                distance_sensor_broadcaster)

        self.tof_sensor = self.robot.getDistanceSensor('tof')
        self.tof_sensor.enable(self.period.value)
        self.tof_publisher = self.create_publisher(Range, '/tof', 1)
        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

        self.tof_broadcaster = StaticTransformBroadcaster(self)
        tof_transform = TransformStamped()
        tof_transform.header.stamp = get_ros_stamp()
        tof_transform.header.frame_id = "base_link"
        tof_transform.child_frame_id = "tof"
        tof_transform.transform.rotation.x = 0.0
        tof_transform.transform.rotation.y = 0.0
        tof_transform.transform.rotation.z = 0.0
        tof_transform.transform.rotation.w = 1.0
        tof_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER
        tof_transform.transform.translation.y = 0.0
        tof_transform.transform.translation.z = 0.0
        self.tof_broadcaster.sendTransform(tof_transform)

        # Initialize camera
        self.camera = self.robot.getCamera('camera')
        self.camera.enable(self.camera_period)
        self.camera_publisher = self.create_publisher(
            Image, '/image_raw', 10)
        self.create_timer(self.camera_period / 1000, self.camera_callback)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, '/camera_info', 10)

        # Initialize binary LEDs
        self.binary_leds = []
        self.binary_led_subscribers = []
        for i in range(NB_BINARY_LEDS):
            index = i * 2
            led = self.robot.getLED('led{}'.format(index))
            led_subscriber = self.create_subscription(
                Bool,
                '/led{}'.format(index),
                partial(self.on_binary_led_callback, index=i),
                10
            )
            self.binary_leds.append(led)
            self.binary_led_subscribers.append(led_subscriber)

        # Initialize RGB LEDs
        self.rgb_leds = []
        self.rgb_led_subscribers = []
        for i in range(NB_RGB_LEDS):
            index = i * 2 + 1
            led = self.robot.getLED('led{}'.format(index))
            led_subscriber = self.create_subscription(
                Int32,
                '/led{}'.format(index),
                partial(self.on_rgb_led_callback, index=i),
                10
            )
            self.rgb_leds.append(led)
            self.rgb_led_subscribers.append(led_subscriber)

        # Initialize Light sensors
        self.light_sensors = []
        self.light_publishers = []
        self.light_sensor_broadcasters = []
        for i in range(NB_LIGHT_SENSORS):
            light_sensor = self.robot.getLightSensor(f'ls{i}')
            light_sensor.enable(self.period.value)
            light_publisher = self.create_publisher(Illuminance, f'/ls{i}', 1)
            self.light_publishers.append(light_publisher)
            self.light_sensors.append(light_sensor)

            light_sensor_broadcaster = StaticTransformBroadcaster(self)
            light_transform = TransformStamped()
            light_transform.header.stamp = get_ros_stamp()
            light_transform.header.frame_id = "base_link"
            light_transform.child_frame_id = "ls" + str(i)
            light_transform.transform.rotation = euler_to_quaternion(
                0, 0, DISTANCE_SENSOR_ANGLE[i])
            light_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER * \
                cos(DISTANCE_SENSOR_ANGLE[i])
            light_transform.transform.translation.y = SENSOR_DIST_FROM_CENTER * \
                sin(DISTANCE_SENSOR_ANGLE[i])
            light_transform.transform.translation.z = 0.0
            light_sensor_broadcaster.sendTransform(light_transform)
            self.light_sensor_broadcasters.append(light_sensor_broadcaster)

        # Static tf broadcaster: Laser
        self.laser_broadcaster = StaticTransformBroadcaster(self)
        laser_transform = TransformStamped()
        laser_transform.header.stamp = get_ros_stamp()
        laser_transform.header.frame_id = "base_link"
        laser_transform.child_frame_id = "laser_scanner"
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.0
        self.laser_broadcaster.sendTransform(laser_transform)

        # Main loop
        self.create_timer(self.period.value / 1000, self.step_callback)

        # Transforms
        self.tf_broadcaster = TransformBroadcaster(self)

    def on_rgb_led_callback(self, msg, index):
        self.rgb_leds[index].set(msg.data)

    def on_binary_led_callback(self, msg, index):
        value = 1 if msg.data else 0
        self.binary_leds[index].set(value)

    def reset_odometry(self):
        self.prev_left_wheel_ticks = 0
        self.prev_right_wheel_ticks = 0
        self.prev_position = (0.0, 0.0)
        self.prev_angle = 0.0

    def on_param_changed(self, params):
        result = SetParametersResult()
        result.successful = True

        for param in params:
            if param.name == "wheel_radius":
                self.reset_odometry()
                self.wheel_radius = param.value
            elif param.name == "wheel_distance":
                self.reset_odometry()
                self.wheel_distance = param.value

        return result

    def step_callback(self):
        self.robot.step(self.period.value)
        stamp = get_ros_stamp()

        self.publish_odometry_data(stamp)
        self.publish_distance_data(stamp)
        self.publish_light_data(stamp)
        self.publish_ground_sensor_data(stamp)

    def publish_ground_sensor_data(self, stamp):
        for idx in self.ground_sensors.keys():
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = idx
            msg.field_of_view = self.ground_sensors[idx].getAperture()
            msg.min_range = GROUND_MIN_RANGE
            msg.max_range = GROUND_MAX_RANGE
            msg.range = interpolate_table(
                self.ground_sensors[idx].getValue(), GROUND_TABLE)
            msg.radiation_type = Range.INFRARED
            self.ground_sensor_publishers[idx].publish(msg)

    def cmd_vel_callback(self, twist):
        self.get_logger().info('Message received')
        left_velocity = (2.0 * twist.linear.x - twist.angular.z *
                         self.wheel_distance) / (2.0 * self.wheel_radius)
        right_velocity = (2.0 * twist.linear.x + twist.angular.z *
                          self.wheel_distance) / (2.0 * self.wheel_radius)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def publish_odometry_data(self, stamp):
        encoder_period_s = self.period.value / 1000.0
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()

        # Calculate velocities
        v_left_rad = (left_wheel_ticks -
                      self.prev_left_wheel_ticks) / encoder_period_s
        v_right_rad = (right_wheel_ticks -
                       self.prev_right_wheel_ticks) / encoder_period_s
        v_left = v_left_rad * self.wheel_radius
        v_right = v_right_rad * self.wheel_radius
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.wheel_distance

        # Calculate position & angle
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(self.prev_angle)
        k01 = v * sin(self.prev_angle)
        k02 = omega
        k10 = v * cos(self.prev_angle + encoder_period_s * k02 / 2)
        k11 = v * sin(self.prev_angle + encoder_period_s * k02 / 2)
        k12 = omega
        k20 = v * cos(self.prev_angle + encoder_period_s * k12 / 2)
        k21 = v * sin(self.prev_angle + encoder_period_s * k12 / 2)
        k22 = omega
        k30 = v * cos(self.prev_angle + encoder_period_s * k22 / 2)
        k31 = v * sin(self.prev_angle + encoder_period_s * k22 / 2)
        k32 = omega
        position = [
            self.prev_position[0] + (encoder_period_s / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self.prev_position[1] + (encoder_period_s / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self.prev_angle + \
            (encoder_period_s / 6) * (k02 + 2 * (k12 + k22) + k32)

        # Update variables
        self.prev_position = position.copy()
        self.prev_angle = angle
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks

        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.orientation = euler_to_quaternion(0, 0, angle)
        self.odometry_publisher.publish(msg)

        # Pack & publish transforms
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation = euler_to_quaternion(0, 0, angle)
        self.tf_broadcaster.sendTransform(tf)

    def publish_light_data(self, stamp):
        for light_publisher, light_sensor in zip(self.light_publishers, self.light_sensors):
            msg = Illuminance()
            msg.header.stamp = stamp
            msg.illuminance = interpolate_table(
                light_sensor.getValue(), LIGHT_TABLE) * IRRADIANCE_TO_ILLUMINANCE
            msg.variance = 0.1
            light_publisher.publish(msg)

    def publish_distance_data(self, stamp):
        dists = [OUT_OF_RANGE] * NB_INFRARED_SENSORS
        dist_tof = OUT_OF_RANGE

        # Calculate distances
        for i, key in enumerate(self.distance_sensors):
            dists[i] = interpolate_table(
                self.distance_sensors[key].getValue(), DISTANCE_TABLE)
        dist_tof = interpolate_table(self.tof_sensor.getValue(), TOF_TABLE)

        # Publish range
        for i, key in enumerate(self.distance_sensors):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = key
            msg.field_of_view = self.distance_sensors[key].getAperture()
            msg.min_range = INFRARED_MIN_RANGE
            msg.max_range = INFRARED_MAX_RANGE
            msg.range = dists[i]
            msg.radiation_type = Range.INFRARED
            self.distance_sensor_publishers[key].publish(msg)
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = 'tof'
        msg.field_of_view = self.tof_sensor.getAperture()
        msg.min_range = TOF_MAX_RANGE
        msg.max_range = TOF_MIN_RANGE
        msg.range = dist_tof
        msg.radiation_type = Range.INFRARED
        self.tof_publisher.publish(msg)

        # Max range of ToF sensor is 2m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        msg = LaserScan()
        msg.header.frame_id = 'laser_scanner'
        msg.header.stamp = stamp
        msg.angle_min = - 150 * pi / 180
        msg.angle_max = 150 * pi / 180
        msg.angle_increment = 15 * pi / 180
        msg.range_min = SENSOR_DIST_FROM_CENTER + INFRARED_MIN_RANGE
        msg.range_max = SENSOR_DIST_FROM_CENTER + INFRARED_MAX_RANGE
        msg.ranges = [
            dists[3] + SENSOR_DIST_FROM_CENTER,     # -150
            OUT_OF_RANGE,                           # -135
            OUT_OF_RANGE,                           # -120
            OUT_OF_RANGE,                           # -105
            dists[2] + SENSOR_DIST_FROM_CENTER,     # -90
            OUT_OF_RANGE,                           # -75
            OUT_OF_RANGE,                           # -60
            dists[1] + SENSOR_DIST_FROM_CENTER,     # -45
            OUT_OF_RANGE,                           # -30
            dists[0] + SENSOR_DIST_FROM_CENTER,     # -15
            dist_tof + SENSOR_DIST_FROM_CENTER,     # 0
            dists[7] + SENSOR_DIST_FROM_CENTER,     # 15
            OUT_OF_RANGE,                           # 30
            dists[6] + SENSOR_DIST_FROM_CENTER,     # 45
            OUT_OF_RANGE,                           # 60
            OUT_OF_RANGE,                           # 75
            dists[5] + SENSOR_DIST_FROM_CENTER,     # 90
            OUT_OF_RANGE,                           # 105
            OUT_OF_RANGE,                           # 120
            OUT_OF_RANGE,                           # 135
            dists[4] + SENSOR_DIST_FROM_CENTER,     # 150
        ]
        self.laser_publisher.publish(msg)

    def imu_callback(self):
        gyro_data = self.gyro.getValues()
        accelerometer_data = self.accelerometer.getValues()

        msg = Imu()
        msg.angular_velocity.x = gyro_data[0]
        msg.angular_velocity.y = gyro_data[1]
        msg.angular_velocity.z = gyro_data[2]
        msg.linear_acceleration.x = accelerometer_data[0]
        msg.linear_acceleration.y = accelerometer_data[1]
        msg.linear_acceleration.z = accelerometer_data[2]
        self.imu_publisher.publish(msg)

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
        msg.header.frame_id = 'camera_frame'
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


def main(args=None):
    rclpy.init(args=args)

    epuck2_controller = EPuckDriver(args=args)

    rclpy.spin(epuck2_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
