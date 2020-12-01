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
from .sensor_device import SensorDevice


class CameraDevice(SensorDevice):
    """
    ROS2 wrapper for Webots Camera node.

    Creates suitable ROS2 interface based on Webots [Camera](https://cyberbotics.com/doc/reference/camera) node instance:

    It allows the following functinalities:
    - Publishes raw image of type `sensor_msgs/Image`
    - Publishes intrinsic camera parameters of type `sensor_msgs/CameraInfo` (latched topic)

    Args:
        node (WebotsNode): The ROS2 node.
        device_key (str): Unique identifier of the device used for configuration.
        wb_device (Camera): Webots node of type Camera.

    Kwargs:
        params (dict): Inherited from `SensorDevice`

    """

    def __init__(self, node, device_key, wb_device, params=None):
        super().__init__(node, device_key, wb_device, params)
        self._camera_info_publisher = None
        self._image_publisher = None

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
        stamp = super().step()
        if not stamp:
            return

        # Publish camera data
        if self._image_publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = self._wb_device.getHeight()
            msg.width = self._wb_device.getWidth()
            msg.is_bigendian = False
            msg.step = self._wb_device.getWidth() * 4
            msg.data = self._wb_device.getImage()
            msg.encoding = 'bgra8'
            self._image_publisher.publish(msg)
        else:
            self._wb_device.disable()
