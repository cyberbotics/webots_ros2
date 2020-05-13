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
                    static_transform = TransformStamped()
                    static_transform.header.stamp = Time(seconds=node.robot.getTime()).to_msg()
                    static_transform.header.frame_id = tf_node.get_parent().get_name()
                    static_transform.child_frame_id = tf_node.get_name()
                    # TODO: Convert rotation matrix to quaternion
                    static_transform.transform.translation.x = tf_node.get_translation()[0]
                    static_transform.transform.translation.y = tf_node.get_translation()[1]
                    static_transform.transform.translation.z = tf_node.get_translation()[2]
                    self.static_transforms.append(static_transform)

    def register(self):
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        self.static_broadcaster.sendTransform(self.static_transforms)
