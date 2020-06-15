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
from .camera_device import CameraDevice
from .led_device import LEDDevice
from .laser_device import LaserDevice
from .distance_sensor_device import DistanceSensorDevice
from .light_sensor_device import LightSensorDevice
from .robot_device import RobotDevice
from webots_ros2_core.utils import append_webots_python_lib_to_path
try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class DeviceManager:
    """Discovers Webots devices and creates corresponding ROS2 topics/services."""

    def __init__(self, node, config=None):
        self._node = node
        self._devices = {}
        config = config or {}

        # Determine default global parameters
        self._auto = config.setdefault('@auto', True)

        # Disable `DeviceManager` if needed
        if not self._auto:
            return

        # Find devices
        self._devices['@robot'] = RobotDevice(node, node.robot, config.get('@robot', None))
        for i in range(node.robot.getNumberOfDevices()):
            wb_device = node.robot.getDeviceByIndex(i)
            if wb_device.getNodeType() == Node.CAMERA:
                self._devices[wb_device.getName()] = CameraDevice(node, wb_device, config.get(wb_device.getName(), None))
            elif wb_device.getNodeType() == Node.LED:
                self._devices[wb_device.getName()] = LEDDevice(node, wb_device, config.get(wb_device.getName(), None))
            elif wb_device.getNodeType() == Node.LIDAR:
                self._devices[wb_device.getName()] = LaserDevice(node, wb_device, config.get(wb_device.getName(), None))
            elif wb_device.getNodeType() == Node.DISTANCE_SENSOR:
                self._devices[wb_device.getName()] = DistanceSensorDevice(node, wb_device, config.get(wb_device.getName(), None))
            elif wb_device.getNodeType() == Node.LIGHT_SENSOR:
                self._devices[wb_device.getName()] = LightSensorDevice(node, wb_device, config.get(wb_device.getName(), None))

        # Verify parameters
        for device_name in config.keys():
            if device_name not in self._devices and device_name not in ['@auto']:
                self._node.get_logger().warn(f'There is no device with name `{device_name}`')

        # Create a loop
        self._node.create_timer(1e-3 * int(node.robot.getBasicTimeStep()), self._callback)

    def _callback(self):
        for device in self._devices.values():
            device.step()
