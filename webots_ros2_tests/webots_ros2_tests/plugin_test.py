# Copyright 1996-2023 Cyberbotics Ltd.
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

"""A simple dummy plugin for testing."""

from std_srvs.srv import Trigger
import rclpy
import rclpy.node


class PluginTest:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        assert properties['parameterExample'] == 'someValue'

        rclpy.init()
        self.__node = rclpy.create_node('plugin_node_test')
        self.__service = self.__node.create_service(Trigger, 'move_forward', self.on_service_call)

    def on_service_call(self, _, response):
        robot_node = self.__robot.getSelf()
        translation_field = robot_node.getField('translation')
        translation_field.setSFVec3f([0.3, 0.0, 0.0])

        response.success = True
        response.message = 'Some success message'
        return response

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
