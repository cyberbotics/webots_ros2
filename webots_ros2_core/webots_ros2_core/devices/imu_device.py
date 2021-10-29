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

"""Webots Accelerometer, Gyro and InertialUnit devices wrapper for ROS2."""

from rclpy.qos import QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from .sensor_device import SensorDevice


class ImuDevice(SensorDevice):
    """
    ROS2 wrapper for Webots Accelerometer, Gyro and InertialUnit node.

    Creates suitable ROS2 interface based on Webots
    [Accelerometer](https://cyberbotics.com/doc/reference/accelerometer),
    [Gyro](https://cyberbotics.com/doc/reference/gyro) and
    [InertialUnit](https://cyberbotics.com/doc/reference/inertialunit) node instances:

    It allows the following functinalities:
    - Combines readings from Accelerometer, Gyro and InertialUnit to ROS topic of type `sensor_msgs/Imu`

    Args:
    ----
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_devices (array): Webots nodes in the following orderd, Accelerometer, Gyro and InertialUnit.
            If device is not available None should be put.

    Kwargs:
        params (dict): Inherited from `SensorDevice`

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)

        self.__accelerometer = wb_device[0]
        self.__gyro = wb_device[1]
        self.__inertial_unit = wb_device[2]

        # Create topics
        self._publisher = None
        if not self._disable:
            qos_sensor_reliable = qos_profile_sensor_data
            qos_sensor_reliable.reliability = QoSReliabilityPolicy.RELIABLE

            self._publisher = self._node.create_publisher(Imu, self._topic_name,
                                                          qos_sensor_reliable)

    def __enable_imu(self):
        for wb_device in self._wb_device:
            if wb_device is not None:
                wb_device.enable(self._timestep)

    def __disable_imu(self):
        for wb_device in self._wb_device:
            if wb_device:
                wb_device.disable()

    def step(self):
        stamp = super().step()
        if not stamp:
            return

        if self._publisher.get_subscription_count() > 0 or self._always_publish:
            self.__enable_imu()
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            if self.__accelerometer:
                raw_data = self.__accelerometer.getValues()
                msg.linear_acceleration.x = interpolate_lookup_table(raw_data[0], self.__accelerometer.getLookupTable())
                msg.linear_acceleration.y = interpolate_lookup_table(raw_data[1], self.__accelerometer.getLookupTable())
                msg.linear_acceleration.z = interpolate_lookup_table(raw_data[2], self.__accelerometer.getLookupTable())
            if self.__gyro:
                raw_data = self.__gyro.getValues()
                msg.angular_velocity.x = interpolate_lookup_table(raw_data[0], self.__gyro.getLookupTable())
                msg.angular_velocity.y = interpolate_lookup_table(raw_data[1], self.__gyro.getLookupTable())
                msg.angular_velocity.z = interpolate_lookup_table(raw_data[2], self.__gyro.getLookupTable())
            if self.__inertial_unit:
                raw_data = self.__inertial_unit.getQuaternion()
                msg.orientation.x = raw_data[0]
                msg.orientation.y = raw_data[1]
                msg.orientation.z = raw_data[2]
                msg.orientation.w = raw_data[3]
            self._publisher.publish(msg)
        else:
            self.__disable_imu()
