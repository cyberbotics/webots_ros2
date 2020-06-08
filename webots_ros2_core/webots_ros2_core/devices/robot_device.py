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

"""Robot device."""

import os
import tempfile
from .device import Device
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg._parameter import Parameter
from rclpy.parameter import ParameterType, ParameterValue


class RobotDevice(Device):
    """Webots + ROS2 Robot wrapper."""

    def __init__(self, node, wb_device, params=None):
        self._node = node
        self._wb_device = wb_device

        # Determine default params
        params = params or {}
        self._topic_name = params.setdefault('topic_name', self._create_topic_name(wb_device))
        self._publish_robot_description = params.setdefault('publish_robot_description', True)

        # Create robot_description publishers if needed
        if self._publish_robot_description:
            urdf = self._wb_device.getUrdf(self.__get_urdf_prefix())
            self.__save_urdf_to_file(urdf)
            self.__set_string_param('robot_state_publisher', 'robot_description', urdf)

    def __save_urdf_to_file(self, urdf):
        """This file supposed to be used for debugging purposes."""
        with open(os.path.join(tempfile.gettempdir(), 'webots_robot.urdf'), 'w') as urdf_file:
            urdf_file.write(urdf)

    def __set_string_param(self, node, name, value):
        self.cli = self._node.create_client(SetParameters, self._node.get_namespace() + node + '/set_parameters')
        self.cli.wait_for_service(timeout_sec=5)
        req = SetParameters.Request()
        param_value = ParameterValue(string_value=value, type=ParameterType.PARAMETER_STRING)
        param = Parameter(name=name, value=param_value)
        req.parameters.append(param)
        self.cli.call_async(req)

    def __get_urdf_prefix(self):
        return self._node.get_namespace()[1:].replace('/', '_')

    def step(self):
        pass
