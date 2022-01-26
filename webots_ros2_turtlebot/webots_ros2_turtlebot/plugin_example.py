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

from std_msgs.msg import Float32
import rclpy
import rclpy.node


class PluginExample:
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.node.Node('plugin_node_example')
        self.__node.get_logger().info('PluginExample: The init() method is called and new node created')

        self.__node.get_logger().info('PluginExample: ')
        self.__node.get_logger().info('  - properties: ' + str(properties))

        self.__node.get_logger().info('  - basic timestep: ' + str(int(webots_node.robot.getBasicTimeStep())))
        self.__node.get_logger().info('  - robot name: ' + str(webots_node.robot.getName()))
        self.__node.get_logger().info('  - is robot? ' + str(not webots_node.robot.getSupervisor()))

        self.__robot = webots_node.robot

        self.__publisher = self.__node.create_publisher(Float32, 'custom_time', 1)
        self.__node.get_logger().info('PluginExample: Publisher created')

    def step(self):
        self.__publisher.publish(Float32(data=self.__robot.getTime()))
