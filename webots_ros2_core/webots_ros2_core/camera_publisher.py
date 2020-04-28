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

"""Camera publisher."""

import sys
from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from webots_ros2_core.utils import append_webots_python_lib_to_path
import transforms3d
try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class CameraPublisherParams:
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


class _WBRCamera:
    """Webots + ROS2 camera wrapper."""

    def __init__(self, device):
        self.device = device
        self.params = CameraPublisherParams()
        self.last_update = -1
        self.camera_info_publisher = None
        self.image_publisher = None


class CameraPublisher():
    """Publish as ROS topics the laser scans of the lidars."""

    def __init__(self, node, parameters=None):
        """
        Initialize the cameras and the topics.
        """
        self.node = node
        self.timestep = int(node.robot.getBasicTimeStep())
        self.wbr_cameras = {}
        parameters = parameters or {}

        # Find camera devices
        for i in range(node.robot.getNumberOfDevices()):
            device = node.robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.CAMERA:
                self.wbr_cameras[device.getName()] = _WBRCamera(device)
                params = parameters[device.getName()] if device.getName() in parameters else CameraPublisherParams()
                params.timestep = params.timestep or self.timestep
                params.topic_name = params.topic_name or device.getName()
                self.wbr_cameras[device.getName()].params = params

        # Verify parameters
        for device_name, params in parameters.items():
            if device_name not in self.wbr_cameras:
                self.node.get_logger().warn(f'There is no camera with name `{device_name}`')

        # Ignore camera if needed
        for device_name, wbr_camera in self.wbr_cameras.items():
            if wbr_camera.params.ignore:
                del self.wbr_cameras[device_name]
                continue

        # Register all devices
        for wbr_camera in self.wbr_cameras.values():
            self._register_camera(wbr_camera)

        # Create a loop
        self.node.create_timer(1e-3 * self.timestep, self._callback)

    def _register_camera(self, wbr_camera):
        wbr_camera.image_publisher = self.node.create_publisher(Image, wbr_camera.params.topic_name + '/image_raw', 1)
        wbr_camera.camera_info_publisher = self.node.create_publisher(
            CameraInfo,
            wbr_camera.params.topic_name + '/camera_info',
            10
        )

    def _callback(self):
        for wbr_camera in self.wbr_cameras.values():
            if self.node.robot.getTime() - wbr_camera.last_update >= wbr_camera.device.getSamplingPeriod() / 1e6:
                self._publish(wbr_camera)
                wbr_camera.last_update = self.node.robot.getTime()

    def _publish(self, wbr_camera):
        """Publish the camera topics with up to date value."""
        stamp = Time(seconds=self.node.robot.getTime() + 1e-3 * self.timestep).to_msg()

        # Publish camera data
        if wbr_camera.image_publisher.get_subscription_count() > 0 or wbr_camera.params.always_publish:
            wbr_camera.device.enable(wbr_camera.params.timestep)

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.height = wbr_camera.device.getHeight()
            msg.width = wbr_camera.device.getWidth()
            msg.is_bigendian = False
            msg.step = wbr_camera.device.getWidth() * 4
            msg.data = wbr_camera.device.getImage()
            msg.encoding = 'bgra8'
            wbr_camera.image_publisher.publish(msg)

            # CameraInfo data
            msg = CameraInfo()
            msg.header.stamp = stamp
            msg.height = wbr_camera.device.getHeight()
            msg.width = wbr_camera.device.getWidth()
            msg.distortion_model = 'plumb_bob'
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [
                wbr_camera.device.getFocalLength(), 0.0, wbr_camera.device.getWidth() / 2,
                0.0, wbr_camera.device.getFocalLength(), wbr_camera.device.getHeight() / 2,
                0.0, 0.0, 1.0
            ]
            msg.p = [
                wbr_camera.device.getFocalLength(), 0.0, wbr_camera.device.getWidth() / 2, 0.0,
                0.0, wbr_camera.device.getFocalLength(), wbr_camera.device.getHeight() / 2, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            wbr_camera.camera_info_publisher.publish(msg)
        else:
            wbr_camera.device.disable()
