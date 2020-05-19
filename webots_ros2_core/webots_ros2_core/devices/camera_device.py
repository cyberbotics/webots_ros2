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

"""Camera device."""

from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from .device import Device


class CameraDevice(Device):
    """Webots + ROS2 camera wrapper."""

    def __init__(self, node, wb_device, params=None):
        self._node = node
        self._wb_device = wb_device
        self._last_update = -1
        self._camera_info_publisher = None
        self._image_publisher = None

        # Determine default params
        params = params or {}
        self._topic_name = params.setdefault('topic_name', self._create_topic_name(wb_device))
        self._timestep = params.setdefault('timestep', int(node.robot.getBasicTimeStep()))
        self._disable = params.setdefault('disable', False)
        self._always_publish = params.setdefault('always_publish', False)

        # Create topics
        if not self._disable:
            self._image_publisher = self._node.create_publisher(
                Image,
                self._topic_name + '/image_raw',
                1
            )
            self._camera_info_publisher = self._node.create_publisher(
                CameraInfo,
                self._topic_name + '/camera_info',
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                )
            )

            # CameraInfo data
            msg = CameraInfo()
            msg.header.stamp = Time(seconds=self._node.robot.getTime()).to_msg()
            msg.height = self._wb_device.getHeight()
            msg.width = self._wb_device.getWidth()
            msg.distortion_model = 'plumb_bob'
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [
                self._wb_device.getFocalLength(), 0.0, self._wb_device.getWidth() / 2,
                0.0, self._wb_device.getFocalLength(), self._wb_device.getHeight() / 2,
                0.0, 0.0, 1.0
            ]
            msg.p = [
                self._wb_device.getFocalLength(), 0.0, self._wb_device.getWidth() / 2, 0.0,
                0.0, self._wb_device.getFocalLength(), self._wb_device.getHeight() / 2, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            self._camera_info_publisher.publish(msg)

            # Load parameters
            camera_period_param = node.declare_parameter(wb_device.getName() + '_period', self._timestep)
            self._camera_period = camera_period_param.value

    def step(self):
        """Publish the camera topics with up to date value."""
        if self._disable:
            return

        if self._node.robot.getTime() - self._last_update < self._timestep / 1e6:
            return
        self._last_update = self._node.robot.getTime()

        stamp = Time(seconds=self._node.robot.getTime()).to_msg()

        # Publish camera data
        if self._image_publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.height = self._wb_device.getHeight()
            msg.width = self._wb_device.getWidth()
            msg.is_bigendian = False
            msg.step = self._wb_device.getWidth() * 4
            msg.data = self._wb_device.getImage()
            msg.encoding = 'bgra8'
            self._image_publisher.publish(msg)
        else:
            self._wb_device.disable()
