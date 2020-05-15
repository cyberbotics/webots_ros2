#!/usr/bin/env python

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

"""Auto discover Webots devices and publish suitable ROS2 topics."""

import sys
from .camera_publisher import CameraPublisher
from .distance_sensor_publisher import DistanceSensorPublisher
from .light_sensor_publisher import LightSensorPublisher
from .tf_publisher import TfPublisher
from webots_ros2_core.utils import append_webots_python_lib_to_path
try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class PublisherManager:
    """Publish as ROS topics the laser scans of the lidars."""

    def __init__(self, node, parameters=None):
        """Initialize the devices and the topics."""
        self._node = node
        self._publishers = {}
        parameters = parameters or {}

        # Transform publisher
        self._publishers['tf'] = TfPublisher(node, parameters.get('tf', None))

        # Find devices
        for i in range(node.robot.getNumberOfDevices()):
            device = node.robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.CAMERA:
                self._publishers[device.getName()] = CameraPublisher(node, device, parameters.get(device.getName(), None))
            elif device.getNodeType() == Node.DISTANCE_SENSOR:
                self._publishers[device.getName()] = DistanceSensorPublisher(
                    node, device, parameters.get(device.getName(), None))
            elif device.getNodeType() == Node.LIGHT_SENSOR:
                self._publishers[device.getName()] = LightSensorPublisher(node, device, parameters.get(device.getName(), None))

        # Verify parameters
        for device_name in parameters.keys():
            if device_name not in self._publishers:
                self._node.get_logger().warn(f'There is no device with name `{device_name}`')

        # Ignore device if needed
        for device_name, publisher in self._publishers.items():
            if publisher.params.ignore:
                del self._publishers[device_name]
                continue

        # Register all devices
        for publisher in self._publishers.values():
            publisher.register()

        # Create a loop
        self._node.create_timer(1e-3 * int(node.robot.getBasicTimeStep()), self._callback)

    def _callback(self):
        for publisher in self._publishers.values():
            publisher.publish()
