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

from webots_ros2_core.webots_node import WebotsNode

import math
import rclpy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TfPublisher(WebotsNode):

    nodeNames = [  # TODO. argument
        'base_link',
        'shoulder_link',
        'upper_arm_link',
        'forearm_link',
        'wrist_1_link',
        'wrist_2_link',
        'wrist_3_link',
        'base',  # TODO: artificial from here
        'ee_link',
        'tool0',
        'world'
    ]

    def __init__(self, args):
        super().__init__('tf_publisher', args)
        self.publisherTimer = self.create_timer(0.001 * self.timestep, self.tf_publisher_callback)
        self.tfPublisher = self.create_publisher(TFMessage, 'tf', 10)
        self.nodes = {}
        for name in TfPublisher.nodeNames:
            node = self.robot.getFromDef(name)
            if node is not None:
                self.nodes[name] = node
            else:
                print(name)  # TODO:log warning

    def tf_publisher_callback(self):
        # Publish TF
        tFMessage = TFMessage()
        for name in self.nodes:
            position = self.nodes[name].getPosition()
            orientation = self.nodes[name].getOrientation()
            transformStamped = TransformStamped()
            time = self.robot.getTime()  # TODO avoid duplication about time
            transformStamped.header.stamp.sec = int(time)
            transformStamped.header.stamp.nanosec = int(round(1000 * (time - int(time))) * 1.0e+6)
            transformStamped.header.frame_id = 'map'
            transformStamped.child_frame_id = name
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


def main(args=None):
    rclpy.init(args=args)

    tfPublisher = TfPublisher(args=args)

    rclpy.spin(tfPublisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
