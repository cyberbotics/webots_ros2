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
from rclpy.time import Time
from std_msgs.msg import Bool, Int32
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import Range, Image, CameraInfo, Imu, LaserScan, Illuminance
from geometry_msgs.msg import TransformStamped
from webots_ros2_core.math_utils import euler_to_quaternion, interpolate_table
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode


GYRO_RAW2DEG = 32768.0 / 250.0
OUT_OF_RANGE = 0.0
TOF_MIN_RANGE = 0.0
TOF_MAX_RANGE = 1.0
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


class EPuckDriver(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__('epuck_driver', args, wheel_distance=DEFAULT_WHEEL_DISTANCE,
                         wheel_radius=DEFAULT_WHEEL_RADIUS)

        self.static_transforms = []

        # Parameters
        camera_period_param = self.declare_parameter("camera_period", self.timestep)
        self.camera_period = camera_period_param.value

        # Initialize IMU
        self.gyro = self.robot.getGyro('gyro')
        if not self.gyro:
            self.get_logger().info('Gyroscope is not present for this e-puck version')
        self.accelerometer = self.robot.getAccelerometer('accelerometer')
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        # Initialize ground sensors
        self.ground_sensors = {}
        self.ground_sensor_publishers = {}
        self.ground_sensor_broadcasters = []
        for i in range(NB_GROUND_SENSORS):
            idx = 'gs{}'.format(i)
            ground_sensor = self.robot.getDistanceSensor(idx)
            if ground_sensor:
                self.ground_sensors[idx] = ground_sensor
                self.ground_sensor_publishers[idx] = self.create_publisher(Range, '/' + idx, 1)

                ground_sensor_transform = TransformStamped()
                ground_sensor_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
                ground_sensor_transform.header.frame_id = "base_link"
                ground_sensor_transform.child_frame_id = "gs" + str(i)
                ground_sensor_transform.transform.rotation = euler_to_quaternion(0, pi/2, 0)
                ground_sensor_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER - 0.005
                ground_sensor_transform.transform.translation.y = 0.009 - i * 0.009
                ground_sensor_transform.transform.translation.z = 0.0
                self.static_transforms.append(ground_sensor_transform)
            else:
                self.get_logger().info('Ground sensor `{}` is not present for this e-puck version'.format(idx))

        # Intialize distance sensors
        self.distance_sensor_publishers = {}
        self.distance_sensors = {}
        for i in range(NB_INFRARED_SENSORS):
            sensor = self.robot.getDistanceSensor('ps{}'.format(i))
            sensor.enable(self.timestep)
            sensor_publisher = self.create_publisher(Range, '/ps{}'.format(i), 10)
            self.distance_sensors['ps{}'.format(i)] = sensor
            self.distance_sensor_publishers['ps{}'.format(i)] = sensor_publisher

            distance_sensor_transform = TransformStamped()
            distance_sensor_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
            distance_sensor_transform.header.frame_id = "base_link"
            distance_sensor_transform.child_frame_id = "ps" + str(i)
            distance_sensor_transform.transform.rotation = euler_to_quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i])
            distance_sensor_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER * cos(DISTANCE_SENSOR_ANGLE[i])
            distance_sensor_transform.transform.translation.y = SENSOR_DIST_FROM_CENTER * sin(DISTANCE_SENSOR_ANGLE[i])
            distance_sensor_transform.transform.translation.z = 0.0
            self.static_transforms.append(distance_sensor_transform)

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

        self.tof_sensor = self.robot.getDistanceSensor('tof')
        if self.tof_sensor:
            self.tof_sensor.enable(self.timestep)
            self.tof_publisher = self.create_publisher(Range, '/tof', 1)
            tof_transform = TransformStamped()
            tof_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
            tof_transform.header.frame_id = "base_link"
            tof_transform.child_frame_id = "tof"
            tof_transform.transform.rotation.x = 0.0
            tof_transform.transform.rotation.y = 0.0
            tof_transform.transform.rotation.z = 0.0
            tof_transform.transform.rotation.w = 1.0
            tof_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER
            tof_transform.transform.translation.y = 0.0
            tof_transform.transform.translation.z = 0.0
            self.static_transforms.append(tof_transform)
        else:
            self.get_logger().info('ToF sensor is not present for this e-puck version')

        # Initialize camera
        self.camera = self.robot.getCamera('camera')
        self.camera_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.create_timer(self.camera_period / 1000, self.camera_callback)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera_info', 10)

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
        for i in range(NB_LIGHT_SENSORS):
            light_sensor = self.robot.getLightSensor(f'ls{i}')
            light_publisher = self.create_publisher(Illuminance, f'/ls{i}', 1)
            self.light_publishers.append(light_publisher)
            self.light_sensors.append(light_sensor)

            light_transform = TransformStamped()
            light_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
            light_transform.header.frame_id = "base_link"
            light_transform.child_frame_id = "ls" + str(i)
            light_transform.transform.rotation = euler_to_quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i])
            light_transform.transform.translation.x = SENSOR_DIST_FROM_CENTER * cos(DISTANCE_SENSOR_ANGLE[i])
            light_transform.transform.translation.y = SENSOR_DIST_FROM_CENTER * sin(DISTANCE_SENSOR_ANGLE[i])
            light_transform.transform.translation.z = 0.0
            self.static_transforms.append(light_transform)

        # Static tf broadcaster: Laser
        laser_transform = TransformStamped()
        laser_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        laser_transform.header.frame_id = "base_link"
        laser_transform.child_frame_id = "laser_scanner"
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.0
        self.static_transforms.append(laser_transform)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(self.static_transforms)

        # Main loop
        self.create_timer(self.timestep / 1000, self.step_callback)

    def on_rgb_led_callback(self, msg, index):
        self.rgb_leds[index].set(msg.data)

    def on_binary_led_callback(self, msg, index):
        value = 1 if msg.data else 0
        self.binary_leds[index].set(value)

    def step_callback(self):
        self.robot.step(self.timestep)
        stamp = Time(seconds=self.robot.getTime()).to_msg()

        self.publish_distance_data(stamp)
        self.publish_light_data(stamp)
        self.publish_ground_sensor_data(stamp)
        self.publish_imu_data(stamp)

    def publish_ground_sensor_data(self, stamp):
        for idx in self.ground_sensors.keys():
            if self.ground_sensor_publishers[idx].get_subscription_count() > 0:
                self.ground_sensors[idx].enable(self.timestep)
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
            else:
                self.ground_sensors[idx].disable()

    def publish_light_data(self, stamp):
        for light_publisher, light_sensor in zip(self.light_publishers, self.light_sensors):
            if light_publisher.get_subscription_count() > 0:
                light_sensor.enable(self.timestep)
                msg = Illuminance()
                msg.header.stamp = stamp
                msg.illuminance = interpolate_table(
                    light_sensor.getValue(), LIGHT_TABLE) * IRRADIANCE_TO_ILLUMINANCE
                msg.variance = 0.1
                light_publisher.publish(msg)
            else:
                light_sensor.disable()

    def publish_distance_data(self, stamp):
        dists = [OUT_OF_RANGE] * NB_INFRARED_SENSORS
        dist_tof = OUT_OF_RANGE

        # Calculate distances
        for i, key in enumerate(self.distance_sensors):
            dists[i] = interpolate_table(
                self.distance_sensors[key].getValue(), DISTANCE_TABLE)

        # Publish range: Infrared
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

        # Publish range: ToF
        if self.tof_sensor:
            dist_tof = interpolate_table(self.tof_sensor.getValue(), TOF_TABLE)
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = 'tof'
            msg.field_of_view = self.tof_sensor.getAperture()
            msg.min_range = TOF_MIN_RANGE
            msg.max_range = TOF_MAX_RANGE
            msg.range = dist_tof
            msg.radiation_type = Range.INFRARED
            self.tof_publisher.publish(msg)

        # Max range of ToF sensor is 2m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        laser_dists = [OUT_OF_RANGE if dist > INFRARED_MAX_RANGE else dist for dist in dists]
        msg = LaserScan()
        msg.header.frame_id = 'laser_scanner'
        msg.header.stamp = stamp
        msg.angle_min = - 150 * pi / 180
        msg.angle_max = 150 * pi / 180
        msg.angle_increment = 15 * pi / 180
        msg.range_min = SENSOR_DIST_FROM_CENTER + INFRARED_MIN_RANGE
        msg.range_max = SENSOR_DIST_FROM_CENTER + TOF_MAX_RANGE
        msg.ranges = [
            laser_dists[3] + SENSOR_DIST_FROM_CENTER,   # -150
            OUT_OF_RANGE,                               # -135
            OUT_OF_RANGE,                               # -120
            OUT_OF_RANGE,                               # -105
            laser_dists[2] + SENSOR_DIST_FROM_CENTER,   # -90
            OUT_OF_RANGE,                               # -75
            OUT_OF_RANGE,                               # -60
            laser_dists[1] + SENSOR_DIST_FROM_CENTER,   # -45
            OUT_OF_RANGE,                               # -30
            laser_dists[0] + SENSOR_DIST_FROM_CENTER,   # -15
            dist_tof + SENSOR_DIST_FROM_CENTER,         # 0
            laser_dists[7] + SENSOR_DIST_FROM_CENTER,   # 15
            OUT_OF_RANGE,                               # 30
            laser_dists[6] + SENSOR_DIST_FROM_CENTER,   # 45
            OUT_OF_RANGE,                               # 60
            OUT_OF_RANGE,                               # 75
            laser_dists[5] + SENSOR_DIST_FROM_CENTER,   # 90
            OUT_OF_RANGE,                               # 105
            OUT_OF_RANGE,                               # 120
            OUT_OF_RANGE,                               # 135
            laser_dists[4] + SENSOR_DIST_FROM_CENTER,   # 150
        ]
        self.laser_publisher.publish(msg)

    def publish_imu_data(self, stamp):
        if self.imu_publisher.get_subscription_count() > 0:
            msg = Imu()
            msg.header.stamp = stamp

            self.accelerometer.enable(self.timestep)
            accelerometer_data = self.accelerometer.getValues()
            msg.linear_acceleration.x = accelerometer_data[1]
            msg.linear_acceleration.y = - accelerometer_data[0]
            msg.linear_acceleration.z = accelerometer_data[2]

            if self.gyro:
                self.gyro.enable(self.timestep)
                gyro_data = self.gyro.getValues()
                msg.angular_velocity.x = (gyro_data[1] / GYRO_RAW2DEG) * (pi / 180)
                msg.angular_velocity.y = - (gyro_data[0] / GYRO_RAW2DEG) * (pi / 180)
                msg.angular_velocity.z = (gyro_data[2] / GYRO_RAW2DEG) * (pi / 180)

            self.imu_publisher.publish(msg)
        else:
            self.accelerometer.disable()
            if self.gyro:
                self.gyro.disable()

    def camera_callback(self):
        if self.camera_publisher.get_subscription_count() > 0:
            self.camera.enable(self.camera_period)

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
        else:
            self.camera.disable()


def main(args=None):
    rclpy.init(args=args)

    epuck_controller = EPuckDriver(args=args)

    rclpy.spin(epuck_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
