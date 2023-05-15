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

import launch
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from webots_ros2_driver.utils import controller_protocol, controller_ip_address


class WebotsController(ExecuteProcess):
    def __init__(self, output='screen', parameters=None, robot_name='', port='1234', **kwargs):
        webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))

        protocol = controller_protocol()
        ip_address = controller_ip_address() if (protocol == 'tcp') else ''

        robot_name_option = [] if not robot_name else ['--robot-name=' + robot_name]
        ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]

        # Get parameters and check if file (yaml extensions) or if single param
        # single_parameters = " ".join(
        #     [f"-p {key}:={value}" for item in parameters if isinstance(item, dict) for key, value in item.items()]
        # )

        single_parameters = []
        for item in parameters:
            if isinstance(item, dict):
                for key, value in item.items():
                    sublist = [f'{key}:=', value if isinstance(value, Substitution) else TextSubstitution(text=str(value))]
                    single_parameters.append('-p')
                    single_parameters.append(sublist)
        print(single_parameters)

        file_parameters = [item for item in parameters if isinstance(item, str)]

        ros_args = ['--ros-args'] if single_parameters else []
        params_file = ['--params-file'] if file_parameters else []

        single_parameters_options = single_parameters if single_parameters else []
        file_parameters_options = file_parameters if file_parameters else []

        os.environ['WEBOTS_HOME'] = get_package_prefix('webots_ros2_driver')
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
                *single_parameters_options,
                *params_file,
                *file_parameters_options,
            ],
            name=node_name,
            **kwargs
        )

    def execute(self, context: LaunchContext):
        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=send_sigint)
