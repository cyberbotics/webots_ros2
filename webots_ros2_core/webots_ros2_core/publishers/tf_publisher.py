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

"""ROS2 TF publisher."""

from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .publisher import Publisher
import math


class _TfDescriptor:
    def __init__(self,
        position_sensor=None,
        initial_rotation=None,
        initial_translation=None,
        axis=None,
        position=None,
        frame_name=None
    ):
        self.position_sensor = position_sensor
        self.initial_rotation = initial_rotation
        self.initial_translation = initial_translation
        self.axis = axis
        self.position = position
        self.frame_name = frame_name


class TfPublisherParams:
    def __init__(
        self,
        timestep=None,
        ignore=False
    ):
        self.timestep = timestep
        self.ignore = ignore


class TfPublisher(Publisher):
    def __init__(self, node, params=None):
        self._node = node
        self._last_update = -1
        self.static_transforms = []
        self.initial_dynamic_transforms = []
        self._tf_descriptions = []

        self.params = params or TfPublisherParams()
        self.params.timestep = self.params.timestep or int(node.robot.getBasicTimeStep())

        tf_queue = [node.robot.getTransformTree()]
        while len(tf_queue) > 0:
            tf_node = tf_queue.pop()
            tf_queue += tf_node.get_children()

            if not tf_node.get_parent():
                continue

            if tf_node.get_type() == node.robot.TF_NODE_LINK:
                rotation = tf_node.get_rotation()
                translation = tf_node.get_translation()
                tf_stamped = TransformStamped()
                tf_stamped.header.stamp = Time(seconds=node.robot.getTime()).to_msg()
                tf_stamped.header.frame_id = self._create_frame_name(tf_node.get_parent())
                tf_stamped.child_frame_id = self._create_frame_name(tf_node)
                tf_stamped.transform.rotation.w = math.sqrt(1.0 + rotation[0] + rotation[4] + rotation[8]) / 2.0
                tf_stamped.transform.rotation.x = (rotation[7] - rotation[5]) / (4.0 * tf_stamped.transform.rotation.w)
                tf_stamped.transform.rotation.y = (rotation[2] - rotation[6]) / (4.0 * tf_stamped.transform.rotation.w)
                tf_stamped.transform.rotation.z = (rotation[3] - rotation[1]) / (4.0 * tf_stamped.transform.rotation.w)
                tf_stamped.transform.translation.x = translation[0]
                tf_stamped.transform.translation.y = translation[1]
                tf_stamped.transform.translation.z = translation[2]

                if tf_node.get_parent().get_type() == node.robot.TF_NODE_LINK:
                    self.static_transforms.append(tf_stamped)
                elif tf_node.get_parent().get_type() == node.robot.TF_NODE_JOINT:
                    self.initial_dynamic_transforms.append(tf_stamped)
                    position_sensor_name = tf_node.get_parent().get_position_sensor_name()
                    if position_sensor_name:
                        position_sensor = node.robot.getPositionSensor(position_sensor_name)
                        position_sensor.enable(node.timestep)
                        self._tf_descriptions.append(_TfDescriptor(
                            frame_name=self._create_frame_name(tf_node),
                            position_sensor=position_sensor,
                            initial_rotation=rotation,
                            initial_translation=translation,
                            axis=tf_node.get_parent().get_axis(),
                            position=tf_node.get_parent().get_position()
                        ))

    def _create_frame_name(self, tf_node):
        if not tf_node.get_parent():
            return 'base_link'
        return tf_node.get_name() if tf_node.is_link_device() else tf_node.get_name() + '_solid'

    def publish(self):
        if self._node.robot.getTime() - self._last_update < self.params.timestep / 1e6:
            return
        self._last_update = self._node.robot.getTime()

        stamp = Time(seconds=self._node.robot.getTime() + 1e-3 * int(self._node.robot.getBasicTimeStep())).to_msg()

        # Publish distance sensor data
        for tf_description in self._tf_descriptions:
            value = tf_description.position_sensor.getValue()
            self._node.get_logger().info(str(value))

    def register(self):
        self.static_broadcaster = StaticTransformBroadcaster(self._node)
        self.static_broadcaster.sendTransform(self.static_transforms)
        self.broadcaster = TransformBroadcaster(self._node)
        self.static_broadcaster.sendTransform(self.initial_dynamic_transforms)
