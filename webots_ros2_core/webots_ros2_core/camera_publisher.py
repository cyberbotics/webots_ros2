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
from tf2_msgs.msg import TFMessage
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
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
        always_publish=False
    ):
        self.timestep = timestep
        self.topic_name = topic_name
        self.always_publish = always_publish


class RosCamera:
    def __init__(self, device):
        self.device = device
        self.params = CameraPublisherParams()
        self.last_update = -1
        self.camera_info_publisher = None
        self.image_publisher = None


class CameraPublisher():
    """Publish as ROS topics the laser scans of the lidars."""

    def __init__(self, node, parameters={}):
        """
        Initialize the cameras and the topics.
        """
        self.node = node
        self.timestep = int(node.robot.getBasicTimeStep())
        self.cameras = {}

        # Find camera devices
        for i in range(node.robot.getNumberOfDevices()):
            device = node.robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.CAMERA:
                self.cameras[device.getName()] = RosCamera(device)

        # Assign parameters
        for device_name, params in parameters.items():
            if device_name not in self.cameras:
                raise NameError(f'There is no camera with name `{device_name}`')
            if not params.timestep:
                params.timestep = self.timestep
            if not params.topic_name:
                params.topic_name = device_name
            self.cameras[device_name].params = params

        # Register all devices
        for camera in self.cameras:
            self.register_camera(camera)

        # Create a loop
        self.node.create_timer(1e-3 * self.timestep, self.callback)

    def register_camera(self, camera):
        camera.device.enable(camera.params.timestep)
        camera.image_publisher = self.node.create_publisher(Image, camera.params.topic_name + '/image_raw', 1)
        camera.camera_info_publisher = self.node.create_publisher(CameraInfo, camera.paprams.topic_name + '/camera_info', 10)


    def callback(self):
        for camera in self.cameras:
            if self.node.robot.getTime() - camera.last_update >= camera.device.getSamplingPeriod():
                self.publish(camera)

    def publish(self, camera):
        """Publish the camera topics with up to date value."""
        stamp = Time(seconds=self.node.robot.getTime() + 1e-3 * self.timestep).to_msg()

        if camera.image_publisher.get_subscription_count() > 0 or camera.params.always_publish:
            camera.device.enable(camera.params.timestep)

            # Image data
            msg = Image()
            msg.header.stamp = stamp
            msg.height = camera.device.getHeight()
            msg.width = camera.device.getWidth()
            msg.is_bigendian = False
            msg.step = camera.device.getWidth() * 4
            msg.data = camera.device.getImage()
            msg.encoding = 'bgra8'
            camera.image_publisher.publish(msg)

            # CameraInfo data
            msg = CameraInfo()
            msg.header.stamp = stamp
            msg.height = camera.device.getHeight()
            msg.width = camera.device.getWidth()
            msg.distortion_model = 'plumb_bob'
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [
                camera.device.getFocalLength(), 0.0, camera.device.getWidth() / 2,
                0.0, camera.device.getFocalLength(), camera.device.getHeight() / 2,
                0.0, 0.0, 1.0
            ]
            msg.p = [
                camera.device.getFocalLength(), 0.0, camera.device.getWidth() / 2, 0.0,
                0.0, camera.device.getFocalLength(), camera.device.getHeight() / 2, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            camera.camera_info_publisher.publish(msg)
        else:
            camera.device.disable()
