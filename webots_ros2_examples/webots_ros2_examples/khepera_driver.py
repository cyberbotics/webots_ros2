# Copyright 1996-2021 Cyberbotics Ltd.
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

"""ROS2 khepera driver."""

from math import pi
import rclpy
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode


SENSOR_DIST_FROM_CENTER = 0.1054


DISTANCE_SENSOR_ANGLE = {
    'rear right infrared sensor': -135 * pi / 180,
    'right infrared sensor': -90 * pi / 180,
    'front right infrared sensor': -45 * pi / 180,
    'front infrared sensor': 0,
    'front left infrared sensor': 45 * pi / 180,
    'left infrared sensor': 90 * pi / 180,
    'rear left infrared sensor': 135 * pi / 180,
    'rear infrared sensor': 180 * pi / 180,
}

ULTRASONIC_SENSOR_ANGLE = {
    'right ultrasonic sensor': -90 * pi / 180,
    'front right ultrasonic sensor': -45 * pi / 180,
    'front ultrasonic sensor': 0,
    'front left ultrasonic sensor': 45 * pi / 180,
    'left ultrasonic sensor': 90 * pi / 180,
}


DEVICE_CONFIG = {
    'camera': {'topic_name': ''},
    'robot': {'publish_base_footprint': True},
    'rear left infrared sensor': {'always_publish': True},
    'left infrared sensor': {'always_publish': True},
    'front left infrared sensor': {'always_publish': True},
    'front infrared sensor': {'always_publish': True},
    'front right infrared sensor': {'always_publish': True},
    'right infrared sensor': {'always_publish': True},
    'rear right infrared sensor': {'always_publish': True},
    'rear infrared sensor': {'always_publish': True},
    'right ultrasonic sensor': {'always_publish': True},
    'front right ultrasonic sensor': {'always_publish': True},
    'front ultrasonic sensor': {'always_publish': True},
    'front left ultrasonic sensor': {'always_publish': True},
    'left ultrasonic sensor': {'always_publish': True}
}


class KheperaDriver(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__(
            'khepera_iv_driver',
            args,
            wheel_distance=0.1054,
            wheel_radius=0.021
        )
        self.start_device_manager(DEVICE_CONFIG)

        # Intialize distance sensors for LaserScan topic
        self.distance_sensors = {}
        for name in DISTANCE_SENSOR_ANGLE.keys():
            sensor = self.robot.getDistanceSensor(name)
            sensor.enable(self.timestep)
            self.distance_sensors[name] = sensor
        for name in ULTRASONIC_SENSOR_ANGLE.keys():
            sensor = self.robot.getDistanceSensor(name)
            sensor.enable(self.timestep)
            self.distance_sensors[name] = sensor

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

        laser_transform = TransformStamped()
        laser_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser_scanner'
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.033

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(laser_transform)

        # Main loop
        self.create_timer(self.timestep / 1000, self.__publish_laserscan_data)

    def __get_ultrasonic_at_angle(self, angle):
        for name, ultrasonic_angle in ULTRASONIC_SENSOR_ANGLE.items():
            if ultrasonic_angle == angle:
                return self.distance_sensors[name]
        return None

    def __publish_laserscan_data(self):
        stamp = Time(seconds=self.robot.getTime()).to_msg()

        lookup_table_infrared = self.distance_sensors['front infrared sensor'].getLookupTable()
        lookup_table_ultrasonic = self.distance_sensors['front ultrasonic sensor'].getLookupTable()

        msg = LaserScan()
        msg.header.frame_id = 'laser_scanner'
        msg.header.stamp = stamp
        msg.angle_min = - 135 * pi / 180
        msg.angle_max = 180 * pi / 180
        msg.angle_increment = 45 * pi / 180
        msg.range_min = min(lookup_table_infrared[0], lookup_table_infrared[-3]) + 0.01
        msg.range_max = max(lookup_table_ultrasonic[0], lookup_table_ultrasonic[-3]) - 0.1

        for name, angle in DISTANCE_SENSOR_ANGLE.items():
            distance = interpolate_lookup_table(self.distance_sensors[name].getValue(), lookup_table_infrared)
            if distance > max(lookup_table_infrared[0], lookup_table_infrared[-3]) - 0.03:
                ultrasonic_sensor = self.__get_ultrasonic_at_angle(angle)
                if ultrasonic_sensor:
                    distance = interpolate_lookup_table(ultrasonic_sensor.getValue(), lookup_table_ultrasonic)
                else:
                    distance = 0

            msg.ranges.append(distance)
        self.laser_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    khepera_controller = KheperaDriver(args=args)

    rclpy.spin(khepera_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
