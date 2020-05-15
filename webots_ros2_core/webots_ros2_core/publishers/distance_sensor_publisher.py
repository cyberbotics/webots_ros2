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

from sensor_msgs.msg import Range
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from webots_ros2_core.math_utils import interpolate_lookup_table
from .publisher import Publisher


class DistanceSensorPublisherParams:
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


class DistanceSensorPublisher(Publisher):
    """Webots + ROS2 distance sensor wrapper."""

    def __init__(self, node, device, params=None):
        self._node = node
        self._device = device
        self._last_update = -1
        self._publisher = None
        self.params = params or DistanceSensorPublisherParams()

        # Determine default params
        self.params.timestep = self.params.timestep or int(node.robot.getBasicTimeStep())
        self.params.topic_name = self.params.topic_name or device.getName()

    def publish(self):
        """Publish the range topics with up to date value."""
        if self._node.robot.getTime() - self._last_update < self._device.getSamplingPeriod() / 1e6:
            return
        self._last_update = self._node.robot.getTime()

        stamp = Time(seconds=self._node.robot.getTime() + 1e-3 * int(self._node.robot.getBasicTimeStep())).to_msg()

        # Publish distance sensor data
        if self._publisher.get_subscription_count() > 0:
            self._device.enable(self.params.timestep)
            msg = Range()
            msg.header.stamp = stamp
            msg.field_of_view = self._device.getAperture()
            msg.min_range = self._device.getMinValue()
            msg.max_range = self._device.getMaxValue()
            msg.range = interpolate_lookup_table(self._device.getValue(), self._device.getLookupTable())
            msg.radiation_type = Range.INFRARED
            self._publisher.publish(msg)
        else:
            self._device.disable()

    def register(self):
        """Register ROS2 publishers when the configuration is ready."""
        self._publisher = self._node.create_publisher(
            Range,
            self.params.topic_name,
            qos_profile_sensor_data
        )
