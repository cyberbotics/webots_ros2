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

def _as_substitution(value):
    return value if isinstance(value, Substitution) else TextSubstitution(text=str(value))

class _ControllerName(Substitution):
    def __init__(self, robot_name):
        self._robot_name = _as_substitution(robot_name)

    def perform(self, context):
        robot_name = context.perform_substitution(self._robot_name)
        return 'webots_controller' + ('_' + robot_name if robot_name else '')

    def describe(self):
        return 'webots_controller name based on robot name'

class WebotsController(ExecuteProcess):
    def __init__(self, output='screen', respawn=False, remappings=[],
                 namespace='', parameters=[], robot_name='', port='1234', **kwargs):
        webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))

        self._robot_name = robot_name
        self._namespace = namespace

        protocol = controller_protocol()
        ip_address = controller_ip_address() if (protocol == 'tcp') else ''

        cmd = [webots_controller]
        self._robot_name_argument_index = None
        if robot_name:
            self._robot_name_argument_index = len(cmd)
            cmd.append(['--robot-name=', _as_substitution(robot_name)])

        cmd.append(['--protocol=', protocol])
        ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]
        cmd.extend(ip_address_option)
        cmd.extend([['--port=', _as_substitution(port)], 'ros2'])

        ros_arguments = []
        namespace_argument_index = None
        for item in remappings:
            key, value = item
            ros_arguments.append('-r')
            ros_arguments.append([_as_substitution(key), ':=', _as_substitution(value)])
        if (namespace):
            namespace_argument_index = len(ros_arguments)
            ros_arguments.append('-r')
            ros_arguments.append(['__ns:=/', _as_substitution(namespace)])
        for item in parameters:
            if isinstance(item, dict):
                for key, value in item.items():
                    parameter = [_as_substitution(key), ':=', _as_substitution(value)]
                    ros_arguments.append('-p')
                    ros_arguments.append(parameter)
        file_parameters = [item for item in parameters if isinstance(item, (str, Substitution))]

        ros_args = ['--ros-args'] if ros_arguments or file_parameters else []
        params_file = ['--params-file'] if file_parameters else []
        self._ros_args_index = len(cmd) if ros_args else None
        self._remove_ros_args_if_namespace_empty = (
            namespace_argument_index == 0 and len(ros_arguments) == 2 and not file_parameters
        )
        cmd.extend(ros_args)
        self._namespace_argument_indices = None
        if namespace_argument_index is not None:
            self._namespace_argument_indices = (
                len(cmd) + namespace_argument_index,
                len(cmd) + namespace_argument_index + 1,
            )
        cmd.extend(ros_arguments)
        cmd.extend(params_file)
        cmd.extend(file_parameters)

        if isinstance(robot_name, Substitution):
            node_name = _ControllerName(robot_name)
        else:
            node_name = 'webots_controller' + ('_' + robot_name if robot_name else '')
        super().__init__(
            output=output,
            cmd=cmd,
            name=node_name,
            respawn=respawn,
            # Set WEBOTS_HOME to package directory to load correct controller library
            additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
            **kwargs
        )

    def execute(self, context: LaunchContext):
        indices_to_remove = []
        robot_name = context.perform_substitution(_as_substitution(self._robot_name))
        if not robot_name and self._robot_name_argument_index is not None:
            indices_to_remove.append(self._robot_name_argument_index)
            self._robot_name_argument_index = None

        if self._namespace_argument_indices is not None:
            namespace = context.perform_substitution(_as_substitution(self._namespace))
            if not namespace:
                indices_to_remove.extend(self._namespace_argument_indices)
                self._namespace_argument_indices = None
                if self._remove_ros_args_if_namespace_empty:
                    indices_to_remove.append(self._ros_args_index)
                    self._ros_args_index = None

        for index in sorted(indices_to_remove, reverse=True):
            del self.cmd[index]

        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=send_sigint)
