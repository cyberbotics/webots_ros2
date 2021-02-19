# Copyright 2021 Intelligent Robotics Labs URJC
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

"""RangeFinder device."""

import numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from .sensor_device import SensorDevice


class RangeFinderDevice(SensorDevice):
    """
    ROS2 wrapper for Webots RangeFinder node.

    Creates suitable ROS2 interface based on Webots
    [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) node instance:

    It allows the following functinalities:
    - Publishes raw depth image of type `sensor_msgs/Image`

    Args:
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (RangeFinder): Webots node of type RangeFinder.

    Kwargs:
        params (dict): Inherited from `SensorDevice`

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)
        self._image_publisher = None

        # Create topics
        if not self._disable:
            self._image_publisher = self._node.create_publisher(
                Image,
                self._topic_name + '/image_depth',
                qos_profile_sensor_data
            )

            # Load parameters
            camera_period_param = node.declare_parameter(wb_device.getName() + '_period', self._timestep)
            self._camera_period = camera_period_param.value

    def step(self):
        stamp = super().step()
        if not stamp:
            return
        # Publish camera data
        if self._image_publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = self._wb_device.getHeight()
            msg.width = self._wb_device.getWidth()
            msg.is_bigendian = False
            msg.step = self._wb_device.getWidth() * 4

            if self._wb_device.getRangeImage() is None:
                return
            image_array = np.array(self._wb_device.getRangeImage(), dtype="float32")

            # We pass `data` directly to we avoid using `data` setter.
            # Otherwise ROS2 converts data to `array.array` which slows down the simulation as it copies memory internally.
            # Both, `bytearray` and `array.array`, implement Python buffer protocol, so we should not see unpredictable
            # behavior.
            # deepcode ignore W0212: Avoid conversion from `bytearray` to `array.array`.
            msg._data = image_array.tobytes()
            msg.encoding = '32FC1'

            self._image_publisher.publish(msg)

        else:
            self._wb_device.disable()
