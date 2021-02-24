#!/usr/bin/env python

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

"""Auto discover Webots devices and publish suitable ROS2 topics."""

from .camera_device import CameraDevice
from .range_finder_device import RangeFinderDevice
from .led_device import LEDDevice
from .lidar_device import LidarDevice
from .distance_sensor_device import DistanceSensorDevice
from .light_sensor_device import LightSensorDevice
from .robot_device import RobotDevice
from .imu_device import ImuDevice
from .gps_device import GpsDevice
from webots_ros2_core.webots.controller import Node


class DeviceManager:
    """Discovers Webots devices and creates corresponding ROS2 topics/services."""

    def __init__(self, node, config=None):
        self.__node = node
        self.__devices = {}
        self.__config = config or {}
        self.__wb_devices = {}

        # Find devices
        self.__devices['robot'] = RobotDevice(node, 'robot', node.robot, self.__config.get('robot', None))
        for i in range(node.robot.getNumberOfDevices()):
            wb_device = node.robot.getDeviceByIndex(i)
            device_key = wb_device.getName()
            device = None

            # Create ROS2 wrapped device
            if wb_device.getNodeType() == Node.CAMERA:
                device = CameraDevice(node, device_key, wb_device, self.__config.get(device_key, None))
            if wb_device.getNodeType() == Node.RANGE_FINDER:
                device = RangeFinderDevice(node, device_key, wb_device, self.__config.get(device_key, None))
            elif wb_device.getNodeType() == Node.LED:
                device = LEDDevice(node, device_key, wb_device, self.__config.get(device_key, None))
            elif wb_device.getNodeType() == Node.LIDAR:
                device = LidarDevice(node, device_key, wb_device, self.__config.get(device_key, None))
            elif wb_device.getNodeType() == Node.DISTANCE_SENSOR:
                device = DistanceSensorDevice(node, device_key, wb_device, self.__config.get(device_key, None))
            elif wb_device.getNodeType() == Node.LIGHT_SENSOR:
                device = LightSensorDevice(node, device_key, wb_device, self.__config.get(device_key, None))
            elif wb_device.getNodeType() == Node.GPS:
                device = GpsDevice(node, device_key, wb_device, self.__config.get(device_key, None))

            # Add device to the list
            self.__wb_devices[device_key] = wb_device
            if device is not None:
                self.__devices[device_key] = device

        # Multi-Webots-device (insert if not configured + create configured)
        self.__insert_imu_device()
        for device_key in self.__config.keys():
            if self.__is_imu_device(device_key):
                self.__devices[device_key] = ImuDevice(
                    node,
                    device_key,
                    self.__get_imu_wb_devices_from_key(device_key),
                    self.__config.get(device_key, None)
                )

        # Verify parameters
        for device_name in self.__config.keys():
            if device_name not in self.__devices.keys():
                self.__node.get_logger().warn(
                    f'Device `{device_name}` is not considered! The device doesn\'t exist or it is not supported.')

    def step(self):
        for device in self.__devices.values():
            device.step()

    def __is_imu_device(self, device_key):
        return any(self.__get_imu_wb_devices_from_key(device_key))

    def __get_imu_wb_devices_from_key(self, device_key):
        wb_device_names = device_key.split('+')

        accelerometer = None
        inertial_unit = None
        gyro = None

        for wb_device_name in wb_device_names:
            if wb_device_name in self.__wb_devices:
                if self.__wb_devices[wb_device_name].getNodeType() == Node.ACCELEROMETER:
                    accelerometer = self.__wb_devices[wb_device_name]
                elif self.__wb_devices[wb_device_name].getNodeType() == Node.INERTIAL_UNIT:
                    inertial_unit = self.__wb_devices[wb_device_name]
                elif self.__wb_devices[wb_device_name].getNodeType() == Node.GYRO:
                    gyro = self.__wb_devices[wb_device_name]

        return [accelerometer, gyro, inertial_unit]

    def __insert_imu_device(self):
        """Insert Imu device only if there is no Imu device configured and there is only one Imu device in the robot."""
        accelerometers = []
        inertial_units = []
        gyros = []

        # Ignore everything if any Imu is configured
        for config_key in self.__config.keys():
            if self.__is_imu_device(config_key):
                return

        # Classify and add to array
        for i in range(self.__node.robot.getNumberOfDevices()):
            wb_device = self.__node.robot.getDeviceByIndex(i)
            if wb_device.getNodeType() == Node.ACCELEROMETER:
                accelerometers.append(wb_device)
            elif wb_device.getNodeType() == Node.INERTIAL_UNIT:
                inertial_units.append(wb_device)
            elif wb_device.getNodeType() == Node.GYRO:
                gyros.append(wb_device)

        # If there is only one candiate for Imu create a device and insert it to `self.__devices`
        if len(accelerometers) <= 1 and len(inertial_units) <= 1 and len(gyros) <= 1 and \
                (len(accelerometers) + len(inertial_units) + len(gyros)) > 0:
            imu_wb_devices = [
                accelerometers[0] if len(accelerometers) > 0 else None,
                gyros[0] if len(gyros) > 0 else None,
                inertial_units[0] if len(inertial_units) > 0 else None
            ]
            imu_wb_device_name = [wb_device.getName() for wb_device in imu_wb_devices if wb_device]
            device_key = '+'.join(imu_wb_device_name)
            self.__devices[device_key] = ImuDevice(self.__node, device_key, imu_wb_devices, {
                'topic_name': '/imu',
                'frame_id': imu_wb_device_name[0]
            })
