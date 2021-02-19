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

"""Webots LightSensor device wrapper for ROS2."""

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Illuminance
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from .sensor_device import SensorDevice


# https://ieee-dataport.org/open-access/conversion-guide-solar-irradiance-and-lux-illuminance
IRRADIANCE_TO_ILLUMINANCE = 120


class LightSensorDevice(SensorDevice):
    """
    ROS2 wrapper for Webots LightSensor node.

    Creates suitable ROS2 interface based on Webots [LightSensor](https://cyberbotics.com/doc/reference/lightsensor) node.

    It allows the following functinalities:
    - Publishes range measurements of type `sensor_msgs/Illuminance`

    Args:
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (LightSensor): Webots node of type LightSensor.

    Kwargs:
        params (dict): Inherited from `SensorDevice`

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)

        # Create topics
        self._publisher = None
        if not self._disable:
            self._publisher = self._node.create_publisher(Illuminance, self._topic_name,
                                                          qos_profile_sensor_data)

    def __get_variance(self, raw_value):
        table = self._wb_device.getLookupTable()

        # Find relative standard deviation in lookup table
        relative_std = None
        for i in range(0, len(table) - 3, 3):
            if table[i+1] < raw_value < table[i+3+1] or table[i+1] > raw_value > table[i+3+1]:
                relative_std = table[i+2]
                break
        if relative_std is None:
            if raw_value < table[1]:
                relative_std = table[2]
            else:
                relative_std = table[-1]

        # Calculate variance from the relative standard deviation
        std = interpolate_lookup_table(raw_value, self._wb_device.getLookupTable()) * IRRADIANCE_TO_ILLUMINANCE * relative_std
        return std**2

    def step(self):
        stamp = super().step()
        if not stamp:
            return

        # Publish light sensor data
        if self._publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)
            msg = Illuminance()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.illuminance = interpolate_lookup_table(self._wb_device.getValue(
            ), self._wb_device.getLookupTable()) * IRRADIANCE_TO_ILLUMINANCE
            msg.variance = self.__get_variance(self._wb_device.getValue())
            self._publisher.publish(msg)
        else:
            self._wb_device.disable()
