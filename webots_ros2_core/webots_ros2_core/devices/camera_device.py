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

"""Camera device."""

from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
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
        self._recognition_publisher = None
        self._recognition_webots_publisher = None
        self._image_publisher = None

        # Create topics
        if not self._disable:
            self._image_publisher = self._node.create_publisher(
                Image,
                self._topic_name + '/image_raw',
                qos_profile_sensor_data
            )
            self._camera_info_publisher = self._node.create_publisher(
                CameraInfo,
                self._topic_name + '/camera_info',
                QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                )
            )

            # CameraInfo data
            self.__message_info = CameraInfo()
            self.__message_info.header.stamp = Time(
                seconds=self._node.robot.getTime()).to_msg()
            self.__message_info.height = self._wb_device.getHeight()
            self.__message_info.width = self._wb_device.getWidth()
            self.__message_info.distortion_model = 'plumb_bob'
            focal_length = self._wb_device.getFocalLength()
            if focal_length == 0:
                focal_length = 570.34  # Identical to Orbbec Astra
            self.__message_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.__message_info.r = [
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.__message_info.k = [
                focal_length, 0.0, self._wb_device.getWidth() / 2,
                0.0, focal_length, self._wb_device.getHeight() / 2,
                0.0, 0.0, 1.0
            ]
            self.__message_info.p = [
                focal_length, 0.0, self._wb_device.getWidth() / 2, 0.0,
                0.0, focal_length, self._wb_device.getHeight() / 2, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            self._camera_info_publisher.publish(self.__message_info)

            # Load parameters
            camera_period_param = node.declare_parameter(
                wb_device.getName() + '_period', self._timestep)
            self._camera_period = camera_period_param.value

    def step(self):
        stamp = super().step()
        if not stamp:
            return
        # Publish camera data
        if self._image_publisher.get_subscription_count() > 0 or self._always_publish:
            self._wb_device.enable(self._timestep)
            image = self._wb_device.getImage()

            if image is None:
                return

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = self._wb_device.getHeight()
            msg.width = self._wb_device.getWidth()
            msg.is_bigendian = False
            msg.step = self._wb_device.getWidth() * 4
            # We pass `data` directly to we avoid using `data` setter.
            # Otherwise ROS2 converts data to `array.array` which slows down the simulation as it copies memory internally.
            # Both, `bytearray` and `array.array`, implement Python buffer protocol, so we should not see unpredictable
            # behavior.
            # deepcode ignore W0212: Avoid conversion from `bytearray` to `array.array`.
            msg._data = image
            msg.encoding = 'bgra8'
            self._image_publisher.publish(msg)

            self.__message_info.header.stamp = Time(
                seconds=self._node.robot.getTime()).to_msg()
            self._camera_info_publisher.publish(self.__message_info)

        else:
            self._wb_device.disable()
