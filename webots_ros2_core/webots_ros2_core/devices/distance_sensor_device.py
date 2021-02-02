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

"""Webots DistanceSensor device wrapper for ROS2."""

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from .sensor_device import SensorDevice


class DistanceSensorDevice(SensorDevice):
    """
    ROS2 wrapper for Webots DistanceSensor node.

    Creates suitable ROS2 interface based on Webots [DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor) node.

    It allows the following functinalities:
    - Publishes range measurements of type `sensor_msgs/Range`

    Args:
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (DistanceSensor): Webots node of type DistanceSensor.

    Kwargs:
        params (dict): Inherited from `SensorDevice`

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)
        self._publisher = None
        self._min_range = self.__get_min_value() + self.__get_lower_std()
        self._max_range = self.__get_max_value() - self.__get_upper_std()

        # Create topics
        if not self._disable:
            self._publisher = self._node.create_publisher(Range, self._topic_name,
                                                          qos_profile_sensor_data)

    def __get_max_value(self):
        table = self._wb_device.getLookupTable()
        return max(table[0], table[-3])

    def __get_min_value(self):
        table = self._wb_device.getLookupTable()
        return min(table[0], table[-3])

    def __get_lower_std(self):
        table = self._wb_device.getLookupTable()
        if table[0] < table[-3]:
            return table[2] * table[0]
        return table[-1] * table[-3]

    def __get_upper_std(self):
        table = self._wb_device.getLookupTable()
        if table[0] > table[-3]:
            return table[2] * table[0]
        return table[-1] * table[-3]

    def step(self):
        stamp = super().step()
        if not stamp:
            return

        # Publish distance sensor data
        if self._publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.field_of_view = self._wb_device.getAperture()
            msg.min_range = self._min_range
            msg.max_range = self._max_range
            msg.range = interpolate_lookup_table(self._wb_device.getValue(), self._wb_device.getLookupTable())
            msg.radiation_type = Range.INFRARED
            self._publisher.publish(msg)
        else:
            self._wb_device.disable()
