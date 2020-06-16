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

"""Webots Accelerometer, Gyro and InertialUnit devices wrapper for ROS2."""

from sensor_msgs.msg import Imu
from webots_ros2_core.math_utils import interpolate_lookup_table
from .sensor_device import SensorDevice


class ImuDevice(SensorDevice):
    """
    ROS2 wrapper for Webots LightSensor node.

    Creates suitable ROS2 interface based on Webots Accelerometer, Gyro and InertialUnit node instances:
    https://cyberbotics.com/doc/reference/accelerometer
    https://cyberbotics.com/doc/reference/gyro
    https://cyberbotics.com/doc/reference/inertialunit

    It allows the following functinalities:
    - Combines readings from Accelerometer, Gyro and InertialUnit to ROS topic of type `sensor_msgs/Imu`

    Args:
        node (WebotsNode): The ROS2 node.
        wb_devices (array): Webots node in the following orderd, Accelerometer, Gyro and InertialUnit. 

    Kwargs:
        params (dict): Inherited from `SensorDevice`

    """

    def __init__(self, node, wb_device, params=None):
        print(params)
        super().__init__(node, wb_device, params)

        # Create topics
        self._publisher = None
        if not self._disable:
            self._publisher = self._node.create_publisher(Imu, self._topic_name, 1)

    def step(self):
        stamp = super().step()
        if not stamp:
            return
