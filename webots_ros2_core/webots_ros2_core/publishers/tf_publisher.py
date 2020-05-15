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

import numpy as np
import transforms3d
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .publisher import Publisher


class _TfDescriptor:
    def __init__(self,
        position_sensor=None,
        initial_rotation=None,
        initial_translation=None,
        axis=None,
        position=None,
        frame_name=None,
        parent_frame_name=None
    ):
        self.position_sensor = position_sensor
        self.initial_rotation = initial_rotation
        self.initial_translation = initial_translation
        self.axis = axis
        self.position = position
        self.frame_name = frame_name
        self.parent_frame_name = parent_frame_name


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
        self._static_transforms = []
        self._initial_dynamic_transforms = []
        self._tf_descriptions = []
        self._static_broadcaster = None
        self._broadcaster = None

        self.params = params or TfPublisherParams()
        self.params.timestep = self.params.timestep or int(node.robot.getBasicTimeStep())

        tf_queue = [node.robot.getTransformTree()]
        while len(tf_queue) > 0:
            tf_node = tf_queue.pop()
            tf_queue += tf_node.get_children()

            if not tf_node.get_parent():
                continue

            if tf_node.get_type() == node.robot.TF_NODE_LINK:
                tf_stamped = TransformStamped()

                rotation = transforms3d.quaternions.mat2quat(np.array(tf_node.get_rotation()).reshape(3, 3))
                translation = tf_node.get_translation()
                frame_name = self._create_frame_name(tf_node)
                parent_frame_name = None

                if tf_node.get_parent().get_type() == node.robot.TF_NODE_LINK:
                    parent_frame_name = self._create_frame_name(tf_node.get_parent())
                    self._static_transforms.append(tf_stamped)
                elif tf_node.get_parent().get_type() == node.robot.TF_NODE_JOINT:
                    parent_frame_name = self._create_frame_name(tf_node.get_parent().get_parent())
                    self._initial_dynamic_transforms.append(tf_stamped)
                    position_sensor_name = tf_node.get_parent().get_position_sensor_name()
                    if position_sensor_name:
                        position_sensor = node.robot.getPositionSensor(position_sensor_name)
                        position_sensor.enable(node.timestep)
                        self._tf_descriptions.append(_TfDescriptor(
                            frame_name=frame_name,
                            parent_frame_name=parent_frame_name,
                            position_sensor=position_sensor,
                            initial_rotation=rotation,
                            initial_translation=translation,
                            axis=tf_node.get_parent().get_axis(),
                            position=tf_node.get_parent().get_position()
                        ))

                tf_stamped.header.stamp = Time(seconds=node.robot.getTime()).to_msg()
                tf_stamped.header.frame_id = parent_frame_name
                tf_stamped.child_frame_id = frame_name
                tf_stamped.transform.rotation.w = rotation[0]
                tf_stamped.transform.rotation.x = rotation[1]
                tf_stamped.transform.rotation.y = rotation[2]
                tf_stamped.transform.rotation.z = rotation[3]
                tf_stamped.transform.translation.x = translation[0]
                tf_stamped.transform.translation.y = translation[1]
                tf_stamped.transform.translation.z = translation[2]

    def _create_frame_name(self, tf_node):
        if not tf_node.get_parent():
            return 'base_link'
        return tf_node.get_name() if tf_node.is_link_device() else tf_node.get_name() + '_solid'

    def publish(self):
        if self._node.robot.getTime() - self._last_update < self.params.timestep / 1e6:
            return
        self._last_update = self._node.robot.getTime()

        for tf_description in self._tf_descriptions:
            value = tf_description.position_sensor.getValue()
            rotation = transforms3d.quaternions.axangle2quat(tf_description.axis, value)

            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = Time(seconds=self._node.robot.getTime()).to_msg()
            tf_stamped.header.frame_id = tf_description.parent_frame_name
            tf_stamped.child_frame_id = tf_description.frame_name
            tf_stamped.transform.rotation.w = rotation[0]
            tf_stamped.transform.rotation.x = rotation[1]
            tf_stamped.transform.rotation.y = rotation[2]
            tf_stamped.transform.rotation.z = rotation[3]
            tf_stamped.transform.translation.x = tf_description.initial_translation[0]
            tf_stamped.transform.translation.y = tf_description.initial_translation[1]
            tf_stamped.transform.translation.z = tf_description.initial_translation[2]
            self._broadcaster.sendTransform(tf_stamped)

    def register(self):
        self._static_broadcaster = StaticTransformBroadcaster(self._node)
        self._static_broadcaster.sendTransform(self._static_transforms)
        self._broadcaster = TransformBroadcaster(self._node)
        self._broadcaster.sendTransform(self._initial_dynamic_transforms)
