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
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from webots_ros2_driver.utils import controller_protocol, controller_ip_address


class _ConditionalSubstitution(Substitution):
    def __init__(self, *, condition, false_value='', true_value=''):
        self.__condition = condition if isinstance(condition, Substitution) else TextSubstitution(text=str(condition))
        self.__false_value = false_value if isinstance(false_value, Substitution) else TextSubstitution(text=false_value)
        self.__true_value = true_value if isinstance(true_value, Substitution) else TextSubstitution(text=true_value)

    def perform(self, context):
        if context.perform_substitution(self.__condition).lower() in ['false', '0', '']:
            return context.perform_substitution(self.__false_value)
        return context.perform_substitution(self.__true_value)


class WebotsController(ExecuteProcess):
    def __init__(self, output='screen', parameters=None, robot_name='', port='1234', **kwargs):
        webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))

        protocol = controller_protocol()
        ip_address = controller_ip_address() if (protocol == 'tcp') else ''

        robot_name_option = _ConditionalSubstitution(condition=robot_name, true_value='--robot-name=' + robot_name)
        ip_address_option = _ConditionalSubstitution(condition=ip_address, true_value='--ip-address=' + ip_address)

        # Get parameters and check if file (yaml extensions) or if single param
        single_parameters = " ".join([f"{key}:={value}" for item in parameters if isinstance(item, dict) for key, value in item.items()])
        file_parameters = " ".join([item for item in parameters if isinstance(item, str)])

        ros_args = _ConditionalSubstitution(condition=single_parameters, true_value='--ros-args')
        params_file = _ConditionalSubstitution(condition=file_parameters, true_value='--params-file')

        single_parameters_options = _ConditionalSubstitution(condition=single_parameters, true_value=single_parameters)
        file_parameters_options = _ConditionalSubstitution(condition=file_parameters, true_value=file_parameters)

        super().__init__(
            output=output,
            cmd=[
                webots_controller,
                robot_name_option,
                ['--protocol=', protocol],
                ip_address_option,
                ['--port=', port],
                'ros2',
                ros_args,
                single_parameters_options,
                params_file,
                file_parameters_options,
            ],
            name='webots_tcp_client',
            **kwargs
        )

    def execute(self, context: LaunchContext):

        # Execute process
        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):

        return super()._shutdown_process(context, send_sigint=send_sigint)
