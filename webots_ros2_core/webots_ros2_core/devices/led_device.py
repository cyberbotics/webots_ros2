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

"""LED device."""

from std_msgs.msg import Int32
from rclpy.qos import QoSReliabilityPolicy, qos_profile_sensor_data
from .device import Device


class LEDDevice(Device):
    """
    ROS2 wrapper for Webots LED node.

    Creates suitable ROS2 interface based on Webots [LED](https://cyberbotics.com/doc/reference/led) node instance:

    It allows the following functinalities:
    - Subscribes to `std_msgs/Int32` and controls LEDs on the robot

    Args:
    ----
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (LED): Webots node of type LED.

    Kwargs:
        params (dict): Dictionary with configuration options in format of::

            dict: {
                'topic_name': str,  # ROS topic name (default will generated from the sensor name)
            }

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)

        # Determine default params
        self._topic_name = self._get_param('topic_name', self._create_topic_name(wb_device))

        qos_sensor_reliable = qos_profile_sensor_data
        qos_sensor_reliable.reliability = QoSReliabilityPolicy.RELIABLE

        # Create subscribers
        self.__led_subscriber = self._node.create_subscription(
            Int32,
            self._topic_name,
            self.__callback,
            qos_sensor_reliable
        )

    def __callback(self, msg):
        self._wb_device.set(msg.data)

    def step(self):
        pass
