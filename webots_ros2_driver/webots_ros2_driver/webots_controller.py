#!/usr/bin/env python

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

"""This launcher simply starts Webots."""

import os

from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from webots_ros2_driver.utils import controller_protocol, controller_ip_address


class WebotsController(ExecuteProcess):
    def __init__(self, output='screen', respawn=False, remappings=[],
                 namespace='', parameters=[], robot_name='', port='1234', **kwargs):
        webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))

        protocol = controller_protocol()
        ip_address = controller_ip_address() if (protocol == 'tcp') else ''

        robot_name_option = [] if not robot_name else ['--robot-name=' + robot_name]
        ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]

        ros_arguments = []
        for item in remappings:
            key, value = item
            remap = f'{key}:={value}'
            ros_arguments.append('-r')
            ros_arguments.append(remap)
        if (namespace):
            remap = f'__ns:=/{namespace}'
            ros_arguments.append('-r')
            ros_arguments.append(remap)
        for item in parameters:
            if isinstance(item, dict):
                for key, value in item.items():
                    parameter = [f'{key}:=', value if isinstance(value, Substitution) else TextSubstitution(text=str(value))]
                    ros_arguments.append('-p')
                    ros_arguments.append(parameter)
        file_parameters = [item for item in parameters if isinstance(item, str)]

        ros_args = ['--ros-args'] if ros_arguments else []
        params_file = ['--params-file'] if file_parameters else []

        node_name = 'webots_controller' + (('_' + robot_name) if robot_name else '')
        super().__init__(
            output=output,
            cmd=[
                webots_controller,
                *robot_name_option,
                ['--protocol=', protocol],
                *ip_address_option,
                ['--port=', port],
                'ros2',
                *ros_args,
                *ros_arguments,
                *params_file,
                *file_parameters,
            ],
            name=node_name,
            respawn=respawn,
            # Set WEBOTS_HOME to package directory to load correct controller library
            additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
            **kwargs
        )

    def execute(self, context: LaunchContext):
        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=send_sigint)
