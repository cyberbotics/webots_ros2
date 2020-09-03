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

"""Laser device."""

from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import transforms3d
from .sensor_device import SensorDevice


class LaserDevice(SensorDevice):
    """
    ROS2 wrapper for Webots Lidar node.

    Creates suitable ROS2 interface based on Webots Lidar node instance:
    https://cyberbotics.com/doc/reference/lidar

    It allows the following functinalities:
    - Publishes range measurements of type `sensor_msgs/LaserScan` for each layer

    Args:
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (Lidar): Webots node of type Lidar.

    Kwargs:
        params (dict): Inherited from `SensorDevice` + the following::

            dict: {
                'noise': int,   # Maximum noise that the sensor can produce, used to compensate the maximum range (default 0.01)
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

        # Create topics
        if wb_device.getNumberOfLayers() > 1:
            for layer_i in range(wb_device.getNumberOfLayers()):
                self.__publishers[self.__get_indexed_topic_name(layer_i)] = node.create_publisher(
                    LaserScan,
                    self.__get_indexed_topic_name(layer_i),
                    1
                )
        else:
            self.__publishers[self._topic_name] = node.create_publisher(LaserScan, self._topic_name, 1)

        # Publish transforms
        self.__static_broadcaster = StaticTransformBroadcaster(node)
        stamp = Time(seconds=self._node.robot.getTime()).to_msg()
        for layer_i in range(wb_device.getNumberOfLayers()):
            static_transform = self.__create_transform(stamp, layer_i)
            self.__static_transforms.append(static_transform)
        self.__static_broadcaster.sendTransform(self.__static_transforms)

    def step(self):
        stamp = super().step()
        if not stamp:
            return

        # Do nothing if no subscribers or `always_publish` is false
        should_publish = self._always_publish
        if not should_publish:
            n_subscribers = 0
            for publisher in self.__publishers.values():
                if publisher.get_subscription_count() > 0:
                    n_subscribers += 1
            if n_subscribers > 0:
                should_publish = True
        if not should_publish:
            self._wb_device.disable()
            return

        # Publish data
        self._wb_device.enable(self._timestep)
        for layer_i in range(self._wb_device.getNumberOfLayers()):
            self.__publish_data(stamp, layer_i)

    def __get_indexed_topic_name(self, layer_i):
        if self._wb_device.getNumberOfLayers() == 1:
            return self._topic_name
        else:
            return self._topic_name + '_' + str(layer_i)

    def __get_indexed_frame_id(self, layer_i):
        if self._wb_device.getNumberOfLayers() == 1:
            return self._frame_id + '_scan'
        else:
            return self._frame_id + '_scan_' + str(layer_i)

    def __create_transform(self, stamp, layer_i=0):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = stamp
        transform_stamped.header.frame_id = self._frame_id
        transform_stamped.child_frame_id = self.__get_indexed_frame_id(layer_i)
        q1 = transforms3d.quaternions.axangle2quat([0, 1, 0], -1.5708)
        q2 = transforms3d.quaternions.axangle2quat([1, 0, 0], 1.5708)
        rotation = transforms3d.quaternions.qmult(q1, q2)
        if self._wb_device.getNumberOfLayers() > 1:
            angle_step = self._wb_device.getVerticalFov() / (self._wb_device.getNumberOfLayers() - 1)
            angle = -0.5 * self._wb_device.getVerticalFov() + layer_i * angle_step
            q3 = transforms3d.quaternions.axangle2quat([0, 0, 1], angle)
            rotation = transforms3d.quaternions.qmult(rotation, q3)
        transform_stamped.transform.rotation.x = rotation[0]
        transform_stamped.transform.rotation.y = rotation[1]
        transform_stamped.transform.rotation.z = rotation[2]
        transform_stamped.transform.rotation.w = rotation[3]
        return transform_stamped

    def __publish_data(self, stamp, layer_i=0):
        """Publish the laser scan topics with up to date value."""
        topic_name = self.__get_indexed_topic_name(layer_i)

        # Publish the actual laser scan
        ranges = self._wb_device.getLayerRangeImage(layer_i)
        if ranges:
            msg = LaserScan()
            msg.header.stamp = stamp
            msg.header.frame_id = self.__get_indexed_frame_id(layer_i)
            msg.angle_min = -0.5 * self._wb_device.getFov()
            msg.angle_max = 0.5 * self._wb_device.getFov()
            msg.angle_increment = self._wb_device.getFov() / (self._wb_device.getHorizontalResolution() - 1)
            msg.scan_time = self._wb_device.getSamplingPeriod() / 1000.0
            msg.range_min = self._wb_device.getMinRange() + self.__noise
            msg.range_max = self._wb_device.getMaxRange() - self.__noise
            msg.ranges = ranges
            self.__publishers[topic_name].publish(msg)
