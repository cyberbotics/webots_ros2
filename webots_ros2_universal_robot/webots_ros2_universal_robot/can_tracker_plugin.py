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

"""Publishes position of the cans (used in system test)."""

from geometry_msgs.msg import Point
import rclpy
import rclpy.node


class CanTrackerPlugin:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.__robot = webots_node.robot
        self.__node = rclpy.node.Node('can_tracker_plugin')
        self.__cans = []
        self.__publishers = []
        for i in range(2):
            self.__cans.append(self.__robot.getFromDef(f'CAN_{i}'))
            self.__publishers.append(self.__node.create_publisher(Point, f'/can_{i}', 1))

    def step(self):
        for i in range(2):
            position = self.__cans[i].getPosition()
            message = Point()
            message.x = position[0]
            message.y = position[1]
            message.z = position[2]
            self.__publishers[i].publish(message)
