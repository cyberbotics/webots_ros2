# Copyright 1996-2019 Cyberbotics Ltd.
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

import math

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

from rclpy.parameter import Parameter


class TfPublisher():
    """This class publishes the transforms of all the Solid nodes of the robots."""

    def __init__(self, robot, node):
        """Initialize the publisher and parse the robot."""
        self.robot = robot
        self.prefix = node.get_parameter_or('prefix',
                                            Parameter('prefix', Parameter.Type.STRING, '')).value
        self.timestep = int(self.robot.getBasicTimeStep())
        self.publisherTimer = node.create_timer(0.001 * self.timestep, self.tf_publisher_callback)
        self.tfPublisher = node.create_publisher(TFMessage, 'tf', 10)
        self.nodes = {}
        # parse the robot structure to detect interesting nodes to publish transforms
        self.parseNode(self.robot.getSelf(), node)

    def parseNode(self, node, rosNode):
        """Recusrive function to parse a node."""
        nameField = node.getProtoField('name')
        endPointField = node.getProtoField('endPoint')
        childrenField = node.getProtoField('children')
        if nameField and nameField.getSFString():
            name = nameField.getSFString()
            if name in self.nodes:
                rosNode.get_logger().warn('Two Solids have the same "%s" name.' % name)
            else:
                self.nodes[name] = node
        if endPointField and endPointField.getSFNode():
            self.parseNode(endPointField.getSFNode(), rosNode)
        if childrenField:
            for i in range(childrenField.getCount()):
                self.parseNode(childrenField.getMFNode(i), rosNode)

    def tf_publisher_callback(self):
        """Publish the current transforms."""
        # Publish TF for the next step
        # we use one step in advance to make sure no sensor data are published before
        stamp = Time(seconds=self.robot.getTime() + 0.001 * self.timestep).to_msg()
        tFMessage = TFMessage()
        for name in self.nodes:
            position = self.nodes[name].getPosition()
            orientation = self.nodes[name].getOrientation()
            transformStamped = TransformStamped()
            transformStamped.header.stamp = stamp
            transformStamped.header.frame_id = 'map'
            transformStamped.child_frame_id = self.prefix + name
            transformStamped.transform.translation.x = position[0]
            transformStamped.transform.translation.y = position[1]
            transformStamped.transform.translation.z = position[2]
            qw = math.sqrt(1.0 + orientation[0] + orientation[4] + orientation[8]) / 2.0
            qx = (orientation[7] - orientation[5]) / (4.0 * qw)
            qy = (orientation[2] - orientation[6]) / (4.0 * qw)
            qz = (orientation[3] - orientation[1]) / (4.0 * qw)
            transformStamped.transform.rotation.x = qx
            transformStamped.transform.rotation.y = qy
            transformStamped.transform.rotation.z = qz
            transformStamped.transform.rotation.w = qw
            tFMessage.transforms.append(transformStamped)
        self.tfPublisher.publish(tFMessage)
