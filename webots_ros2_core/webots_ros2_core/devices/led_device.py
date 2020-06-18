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

"""LED device."""

from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
from .device import Device


class LEDDevice(Device):
    """Webots + ROS2 LED wrapper."""

    def __init__(self, node, device_key, wb_device, params=None):
        self._node = node
        self._device_key = device_key
        self._wb_device = wb_device
        self._last_update = -1

        # Determine default params
        params = params or {}
        self._topic_name = params.setdefault('topic_name', self._create_topic_name(wb_device))

        # Create publishers
        self._led_subscriber = node.create_subscription(
            Int32,
            self._topic_name,
            self._callback,
            qos_profile_sensor_data
        )

    def _callback(self, msg):
        self._wb_device.set(msg.data)

    def step(self):
        pass
