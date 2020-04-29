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

"""LightSensor publisher."""

from sensor_msgs.msg import Illuminance
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from webots_ros2_core.math_utils import interpolate_table
from .publisher import Publisher


IRRADIANCE_TO_ILLUMINANCE = 120
LIGHT_TABLE = [
    [1, 4095],
    [2, 0]
]

class LightSensorPublisherParams:
    def __init__(
        self,
        timestep=None,
        topic_name=None,
        always_publish=False,
        ignore=False
    ):
        self.timestep = timestep
        self.topic_name = topic_name
        self.always_publish = always_publish
        self.ignore = ignore


class LightSensorPublisher(Publisher):
    """Webots + ROS2 camera wrapper."""

    def __init__(self, node, device, params=None):
        self._node = node
        self._device = device
        self._last_update = -1
        self._publisher = None
        self.params = params or LightSensorPublisherParams()

        # Determine default params
        self.params.timestep = self.params.timestep or int(node.robot.getBasicTimeStep())
        self.params.topic_name = self.params.topic_name or device.getName()

    def publish(self):
        """Publish the camera topics with up to date value."""
        if self._node.robot.getTime() - self._last_update < self._device.getSamplingPeriod() / 1e6:
            return
        self._last_update = self._node.robot.getTime()

        stamp = Time(seconds=self._node.robot.getTime() + 1e-3 * int(self._node.robot.getBasicTimeStep())).to_msg()

        # Publish camera data
        if self._publisher.get_subscription_count() > 0:
            self._device.enable(self.params.timestep)
            msg = Illuminance()
            msg.header.stamp = stamp
            msg.illuminance = interpolate_table(
                self._device.getValue(), LIGHT_TABLE) * IRRADIANCE_TO_ILLUMINANCE
            msg.variance = 0.1
            self._publisher.publish(msg)
        else:
            self._device.disable()

    def register(self):
        """Register ROS2 publishers when the configuration is ready."""
        self._publisher = self._node.create_publisher(
            Illuminance,
            self.params.topic_name,
            qos_profile_sensor_data
        )
