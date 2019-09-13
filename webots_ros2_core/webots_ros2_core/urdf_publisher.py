#!/usr/bin/env python

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

"""Simple node publishing the content of an URDF file."""

import argparse
import rclpy

from std_msgs.msg import String

from rclpy.node import Node


class UrdfPublisher(Node):

    def __init__(self, args=None):
        super().__init__('urdf_publisher')
        parser = argparse.ArgumentParser()
        parser.add_argument('--urdf-file', dest='urdfFile', default='',
                            help='Specifies the path to the URDF file.')
        parser.add_argument('--topic-name', dest='topicName', default='robot_description',
                            help='Specifies the name of the published topic.')
        parser.add_argument('--frequency', type=float, dest='frequency', default=1.0,
                            help='Specifies the frequency at which the topic should be published.')
        # use 'parse_known_args' because ROS2 adds a lot of internal arguments
        arguments, unknown = parser.parse_known_args()
        self.clockPublisher = self.create_publisher(String, arguments.topicName)
        self.timer = self.create_timer(1.0 / arguments.frequency, self.timer_callback)
        self.urdfContent = arguments.urdfFile
        with open(arguments.urdfFile, 'r') as f:  #TODO. check existance
            self.urdfContent = f.read()

    def timer_callback(self):
        # Publish clock
        msg = String()
        msg.data = self.urdfContent
        self.clockPublisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    urdfPublisher = UrdfPublisher(args=args)

    rclpy.spin(urdfPublisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
