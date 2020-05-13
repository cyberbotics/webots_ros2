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
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .publisher import Publisher
import math


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
        self.node = node
        self.static_transforms = []

        self.params = params or TfPublisherParams()
        self.params.timestep = self.params.timestep or int(node.robot.getBasicTimeStep())

        tf_queue = [node.robot.getTransformTree()]
        while len(tf_queue) > 0:
            tf_node = tf_queue.pop()
            tf_queue += tf_node.get_children()

            if not tf_node.get_parent():
                continue

            if tf_node.get_type() == node.robot.TF_NODE_LINK:
                if tf_node.get_parent().get_type() == node.robot.TF_NODE_LINK:
                    rotation = tf_node.get_rotation()
                    translation = tf_node.get_translation()
                    
                    static_tf = TransformStamped()
                    static_tf.header.stamp = Time(seconds=node.robot.getTime()).to_msg()
                    static_tf.header.frame_id = self.__create_frame_name(tf_node.get_parent())
                    static_tf.child_frame_id = self.__create_frame_name(tf_node)
                    static_tf.transform.rotation.w = math.sqrt(1.0 + rotation[0] + rotation[4] + rotation[8]) / 2.0
                    static_tf.transform.rotation.x = (rotation[7] - rotation[5]) / (4.0 * static_tf.transform.rotation.w)
                    static_tf.transform.rotation.y = (rotation[2] - rotation[6]) / (4.0 * static_tf.transform.rotation.w)
                    static_tf.transform.rotation.z = (rotation[3] - rotation[1]) / (4.0 * static_tf.transform.rotation.w)
                    static_tf.transform.translation.x = translation[0]
                    static_tf.transform.translation.y = translation[1]
                    static_tf.transform.translation.z = translation[2]
                    self.static_transforms.append(static_tf)

    def __create_frame_name(self, tf_node):
        if not tf_node.get_parent():
            return 'base_link'
        return tf_node.get_name() if tf_node.is_link_device() else tf_node.get_name() + '_solid'

    def register(self):
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        self.static_broadcaster.sendTransform(self.static_transforms)
