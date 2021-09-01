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

"""Webots generic sensor device wrapper for ROS2."""

from rclpy.time import Time
from .device import Device


class SensorDevice(Device):
    """
    ROS2 wrapper for Webots sensor nodes.

    Args:
    ----
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (Device): Webots device node.

    Kwargs:
        params (dict): Dictionary with configuration options in format of::

            dict: {
                'topic_name': str,      # ROS topic name (default will generated from the sensor name)
                'timestep': int,        # Publish period in ms (default is equal to robot's timestep)
                'disable': bool,        # Whether to create ROS interface for this sensor (default false)
                'always_publish': bool, # Publish even if there are no subscribers (default false)
                'frame_id': str,        # Value for `header.frame_id` field (default is the same as device name)
            }

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)

        self._last_update = -1

        reference_device = [single_wb_device for single_wb_device in wb_device if single_wb_device][0] if isinstance(
            wb_device, list) else wb_device

        # Determine default params
        self._topic_name = self._get_param('topic_name', self._create_topic_name(reference_device))
        self._timestep = self._get_param('timestep', int(node.robot.getBasicTimeStep()))
        self._disable = self._get_param('disable', False)
        self._always_publish = self._get_param('always_publish', False)
        self._frame_id = self._get_param('frame_id', reference_device.getName())

    def step(self):
        if self._disable:
            return None

        if self._node.robot.getTime() - self._last_update < self._timestep / 1e3:
            return None
        self._last_update = self._node.robot.getTime()

        return Time(seconds=self._node.robot.getTime()).to_msg()
