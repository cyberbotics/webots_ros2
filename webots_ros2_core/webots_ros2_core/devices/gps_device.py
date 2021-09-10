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

"""Webots GPS device wrapper for ROS2."""

from rclpy.qos import QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PointStamped
from .sensor_device import SensorDevice
from controller import GPS


class GpsDevice(SensorDevice):
    """
    ROS2 wrapper for Webots GPS node.

    Creates suitable ROS2 interface based on Webots
    [GPS](https://cyberbotics.com/doc/reference/gps) node instance:

    It allows the following functinalities:
    - Publishes position measurements of type `sensor_msgs::NavSatFix` if WGS84
    - Publishes position measurements of type `geometry_msgs::PointStamped` if LOCAL

    Args:
    ----
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (Gps): Webots node of type GPS.

    Kwargs:
        params (dict): Inherited from `SensorDevice` + the following::

            dict: {
                'timestep': int,              # Publish period in ms (default 128ms)
            }

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)
        self.__speed_publisher = None
        self.__gps_publisher = None
        self.__coordinate_system = self._wb_device.getCoordinateSystem()

        # Exit if disabled
        if self._disable:
            return

        # Change default timestep
        self._timestep = 128

        qos_sensor_reliable = qos_profile_sensor_data
        qos_sensor_reliable.reliability = QoSReliabilityPolicy.RELIABLE

        # Create topics
        self.__speed_publisher = node.create_publisher(
            Float32, self._topic_name + '/speed', qos_sensor_reliable)

        if self.__coordinate_system == GPS.WGS84:
            self.__gps_publisher = node.create_publisher(
                NavSatFix, self._topic_name + '/gps', qos_sensor_reliable)
        else:
            self.__gps_publisher = node.create_publisher(
                PointStamped, self._topic_name + '/gps', qos_sensor_reliable)

    def step(self):
        stamp = super().step()
        if not stamp:
            return

        if self.__gps_publisher.get_subscription_count() > 0 or \
            self.__speed_publisher.get_subscription_count() > 0 or \
                self._always_publish:
            self._wb_device.enable(self._timestep)
            msg = Float32()
            msg.data = self._wb_device.getSpeed()
            self.__speed_publisher.publish(msg)
            if self.__coordinate_system == GPS.WGS84:
                msg = NavSatFix()
                msg.header.stamp = stamp
                msg.header.frame_id = self._frame_id
                msg.latitude = self._wb_device.getValues()[0]
                msg.longitude = self._wb_device.getValues()[1]
                msg.altitude = self._wb_device.getValues()[2]
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                msg.status.service = NavSatStatus.SERVICE_GPS
                self.__gps_publisher.publish(msg)
            else:
                msg = PointStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = self._frame_id
                msg.point.x = self._wb_device.getValues()[0]
                msg.point.y = self._wb_device.getValues()[1]
                msg.point.z = self._wb_device.getValues()[2]
                self.__gps_publisher.publish(msg)
        else:
            self._wb_device.disable()
