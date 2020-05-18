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

"""Camera plugin."""

from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from .plugin import Plugin


class CameraPluginParams:
    def __init__(
        self,
        timestep=None,
        topic_name=None,
        always_publish=False,
        disable=False
    ):
        self.timestep = timestep
        self.topic_name = topic_name
        self.always_publish = always_publish
        self.disable = disable


class CameraPlugin(Plugin):
    """Webots + ROS2 camera wrapper."""

    def __init__(self, node, device, params=None):
        self._node = node
        self._device = device
        self._last_update = -1
        self._camera_info_plugin = None
        self._image_publisher = None
        self.params = params or CameraPluginParams()

        # Determine default params
        self.params.timestep = self.params.timestep or int(node.robot.getBasicTimeStep())
        self.params.topic_name = self.params.topic_name or device.getName()

        if not self.params.disable:
            self._image_publisher = self._node.create_publisher(
                Image,
                self.params.topic_name + '/image_raw',
                qos_profile_sensor_data
            )
            self._camera_info_publisher = self._node.create_publisher(
                CameraInfo,
                self.params.topic_name + '/camera_info',
                qos_profile_sensor_data
            )
            camera_period_param = node.declare_parameter(device.getName() + '_period', self.params.timestep)
            self._camera_period = camera_period_param.value

    def step(self):
        """Publish the camera topics with up to date value."""
        if self.params.disable:
            return

        if self._node.robot.getTime() - self._last_update < self.params.timestep / 1e6:
            return
        self._last_update = self._node.robot.getTime()

        stamp = Time(seconds=self._node.robot.getTime() + 1e-3 * int(self._node.robot.getBasicTimeStep())).to_msg()

        # Publish camera data
        if self._image_publisher.get_subscription_count() > 0 or self.params.always_publish:
            self._device.enable(self.params.timestep)

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.height = self._device.getHeight()
            msg.width = self._device.getWidth()
            msg.is_bigendian = False
            msg.step = self._device.getWidth() * 4
            msg.data = self._device.getImage()
            msg.encoding = 'bgra8'
            self._image_publisher.publish(msg)

            # CameraInfo data
            msg = CameraInfo()
            msg.header.stamp = stamp
            msg.height = self._device.getHeight()
            msg.width = self._device.getWidth()
            msg.distortion_model = 'plumb_bob'
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [
                self._device.getFocalLength(), 0.0, self._device.getWidth() / 2,
                0.0, self._device.getFocalLength(), self._device.getHeight() / 2,
                0.0, 0.0, 1.0
            ]
            msg.p = [
                self._device.getFocalLength(), 0.0, self._device.getWidth() / 2, 0.0,
                0.0, self._device.getFocalLength(), self._device.getHeight() / 2, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            self._camera_info_publisher.publish(msg)
        else:
            self._device.disable()
