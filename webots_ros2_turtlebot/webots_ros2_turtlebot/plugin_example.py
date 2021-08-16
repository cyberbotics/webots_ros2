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

"""A simple dummy plugin that demonstrates the usage of Python plugins."""

from webots_ros2_driver_webots.controller import Node
from std_msgs.msg import Float32
import rclpy
import rclpy.node


class PluginExample:
    def init(self, webots_node, properties):
        print('PluginExample: The init() method is called')
        print('  - properties:', properties)

        print('  - basic timestep:', int(webots_node.robot.getBasicTimeStep()))
        print('  - robot name:', webots_node.robot.getName())
        print('  - is robot?', webots_node.robot.getType() == Node.ROBOT)

        self.__robot = webots_node.robot

        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.node.Node('plugin_node_example')
        print('PluginExample: Node created')
        self.__publisher = self.__node.create_publisher(Float32, 'custom_time', 1)
        print('PluginExample: Publisher created')

    def step(self):
        self.__publisher.publish(Float32(data=self.__robot.getTime()))
