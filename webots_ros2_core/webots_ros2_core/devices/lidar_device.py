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

"""Lidar device."""

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .sensor_device import SensorDevice


class LidarDevice(SensorDevice):
    """
    ROS2 wrapper for Webots Lidar node.

    Creates suitable ROS2 interface based on Webots [Lidar](https://cyberbotics.com/doc/reference/lidar) node instance:

    It allows the following functinalities:
    - Publishes range measurements of type `sensor_msgs/LaserScan` if 2D Lidar is present
    - Publishes range measurements of type `sensor_msgs/PointCloud2` if 3D Lidar is present

    Args:
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (Lidar): Webots node of type Lidar.

    Kwargs:
        params (dict): Inherited from `SensorDevice` + the following::

            dict: {
                'noise': int,       # Maximum sensor noise used to compensate the maximum range (default 0.01)
                'timestep': int,    # Publish period in ms (default 128ms)
            }

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)
        self.__publishers = {}
        self.__static_transforms = []
        self.__static_broadcaster = None
        self.__noise = self._get_param('noise', 1e-2)

        # Exit if disabled
        if self._disable:
            return

        # Change default timestep
        self._timestep = 128

        # Create topics
        if wb_device.getNumberOfLayers() > 1:
            wb_device.enablePointCloud()
            self.__publisher = node.create_publisher(PointCloud2, self._topic_name,
                                                     qos_profile_sensor_data)
        else:
            self.__publisher = node.create_publisher(LaserScan, self._topic_name,
                                                     qos_profile_sensor_data)
            self.__static_broadcaster = StaticTransformBroadcaster(node)
            transform_stamped = TransformStamped()
            transform_stamped.header.frame_id = self._frame_id
            transform_stamped.child_frame_id = self._frame_id + '_rotated'
            transform_stamped.transform.rotation.x = 0.5
            transform_stamped.transform.rotation.y = 0.5
            transform_stamped.transform.rotation.z = -0.5
            transform_stamped.transform.rotation.w = 0.5
            self.__static_broadcaster.sendTransform(transform_stamped)

    def step(self):
        stamp = super().step()
        if not stamp:
            return

        if self.__publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)
            if self._wb_device.getNumberOfLayers() > 1:
                self.__publish_point_cloud_data(stamp)
            else:
                self.__publish_laser_scan_data(stamp)
        else:
            self._wb_device.disable()

    def __publish_point_cloud_data(self, stamp):
        data = self._wb_device.getPointCloud(data_type='buffer')
        if data:
            msg = PointCloud2()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = 1
            msg.width = self._wb_device.getNumberOfPoints()
            msg.point_step = 20
            msg.row_step = 20 * self._wb_device.getNumberOfPoints()
            msg.is_dense = False
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            msg.is_bigendian = False
            # We pass `data` directly to we avoid using `data` setter.
            # Otherwise ROS2 converts data to `array.array` which slows down the simulation as it copies memory internally.
            # Both, `bytearray` and `array.array`, implement Python buffer protocol, so we should not see unpredictable
            # behavior.
            # deepcode ignore W0212: Avoid conversion from `bytearray` to `array.array`.
            msg._data = data
            self.__publisher.publish(msg)

    def __publish_laser_scan_data(self, stamp):
        """Publish the laser scan topics with up to date value."""
        ranges = self._wb_device.getLayerRangeImage(0)
        if ranges:
            msg = LaserScan()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id + '_rotated'
            msg.angle_min = -0.5 * self._wb_device.getFov()
            msg.angle_max = 0.5 * self._wb_device.getFov()
            msg.angle_increment = self._wb_device.getFov() / (self._wb_device.getHorizontalResolution() - 1)
            msg.scan_time = self._wb_device.getSamplingPeriod() / 1000.0
            msg.range_min = self._wb_device.getMinRange() + self.__noise
            msg.range_max = self._wb_device.getMaxRange() - self.__noise
            msg.ranges = ranges
            self.__publisher.publish(msg)
