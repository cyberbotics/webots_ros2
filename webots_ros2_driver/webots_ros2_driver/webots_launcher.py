#!/usr/bin/env python

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

"""This launcher simply starts Webots."""

import os
import re
import shutil
import subprocess
import sys
import tempfile

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from webots_ros2_driver.utils import (get_webots_home,
                                      handle_webots_installation,
                                      get_wsl_ip_address,
                                      is_wsl)


class _ConditionalSubstitution(Substitution):
    def __init__(self, *, condition, false_value='', true_value=''):
        self.__condition = condition if isinstance(condition, Substitution) else TextSubstitution(text=str(condition))
        self.__false_value = false_value if isinstance(false_value, Substitution) else TextSubstitution(text=false_value)
        self.__true_value = true_value if isinstance(true_value, Substitution) else TextSubstitution(text=true_value)

    def perform(self, context):
        if context.perform_substitution(self.__condition).lower() in ['false', '0', '']:
            return context.perform_substitution(self.__false_value)
        return context.perform_substitution(self.__true_value)


class WebotsLauncher(ExecuteProcess):
    def __init__(self, output='screen', world=None, gui=True, mode='realtime', stream=False, **kwargs):
        if sys.platform == 'win32':
            print(f'WARNING: Native webots_ros2 compatibility with Windows is deprecated and will be removed soon. Please use a WSL (Windows Subsystem for Linux) environment instead.')
            print(f'WARNING: Check https://github.com/cyberbotics/webots_ros2/wiki/Complete-Installation-Guide for more information.')
        self.__is_wsl = is_wsl()

        # Find Webots executable
        webots_path = get_webots_home(show_warning=True)
        if webots_path is None:
            handle_webots_installation()
            webots_path = get_webots_home()
        if self.__is_wsl:
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin', 'webots.exe')
        else:
            webots_path = os.path.join(webots_path, 'webots')

        mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)

        self.__world_copy = tempfile.NamedTemporaryFile(mode='w+', suffix='_world_with_URDF_robot.wbt', delete=False)
        self.__world = world
        if not isinstance(world, Substitution):
            world = TextSubstitution(text=self.__world_copy.name)

        if self.__is_wsl:
            wsl_tmp_path = subprocess.check_output(['wslpath', '-w', self.__world_copy.name]).strip().decode('utf-8')
            world = TextSubstitution(text=wsl_tmp_path)

        no_rendering = _ConditionalSubstitution(condition=gui, false_value='--no-rendering')
        stdout = _ConditionalSubstitution(condition=gui, false_value='--stdout')
        stderr = _ConditionalSubstitution(condition=gui, false_value='--stderr')
        minimize = _ConditionalSubstitution(condition=gui, false_value='--minimize')
        stream_argument = _ConditionalSubstitution(condition=stream, true_value='--stream')
        xvfb_run_prefix = []

        if 'WEBOTS_OFFSCREEN' in os.environ:
            xvfb_run_prefix.append('xvfb-run')
            xvfb_run_prefix.append('--auto-servernum')
            no_rendering = '--no-rendering'

        # no_rendering, stdout, stderr, minimize
        super().__init__(
            output=output,
            cmd=xvfb_run_prefix + [
                webots_path,
                stream_argument,
                no_rendering,
                stdout,
                stderr,
                minimize,
                world,
                '--batch',
                ['--mode=', mode],
            ],
            **kwargs
        )

    def execute(self, context: LaunchContext):
        # User can give a PathJoinSubstitution world or an absolute path world
        if isinstance(self.__world, PathJoinSubstitution):
            world_path = self.__world.perform(context)
            context.launch_configurations['world'] = self.__world_copy.name
        else:
            world_path = self.__world

        shutil.copy2(world_path, self.__world_copy.name)

        # Update relative paths in the world
        with open(self.__world_copy.name, 'r') as file:
            content = file.read()

        for match in re.finditer('\"((?:[^\"]*)\\.(?:jpe?g|png|hdr|obj|stl|dae|wav|mp3|proto))\"', content):
            url_path = match.group(1)

            # Absolute path or Webots relative path or Web paths
            if os.path.isabs(url_path) or url_path.startswith('webots://') or url_path.startswith('http://') or url_path.startswith('https://'):
                continue

            new_url_path = '"' + os.path.split(world_path)[0] + '/' + url_path + '"'
            url_path = '"' + url_path + '"'
            content = content.replace(url_path, new_url_path)

        with open(self.__world_copy.name, 'w') as file:
            file.write(content)

        # Add the Ros2Supervisor
        indent = '  '
        world_file = open(self.__world_copy.name, 'a')
        world_file.write('Robot {\n')
        world_file.write(indent + 'name "Ros2Supervisor"\n')
        world_file.write(indent + 'controller "<extern>"\n')
        world_file.write(indent + 'supervisor TRUE\n')
        world_file.write('}\n')
        world_file.close()

        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        # Remove copy of the world and the corresponding ".wbproj" file
        if self.__world_copy:
            self.__world_copy.close()
            if os.path.isfile(self.__world_copy.name):
                os.unlink(self.__world_copy.name)

            path, file = os.path.split(self.__world_copy.name)
            world_copy_secondary_file = os.path.join(path, '.' + file[:-1] + 'proj')
            if os.path.isfile(world_copy_secondary_file):
                os.unlink(world_copy_secondary_file)
        return super()._shutdown_process(context, send_sigint=send_sigint)


class Ros2SupervisorLauncher(Node):
    def __init__(self, output='screen', respawn=True, **kwargs):
        controller_url = 'tcp://' + get_wsl_ip_address() + ':1234/' if is_wsl() else ''

        # Launch the Ros2Supervisor node
        super().__init__(
            package='webots_ros2_driver',
            executable='ros2_supervisor.py',
            output=output,
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url + 'Ros2Supervisor'},
            respawn=respawn,
            **kwargs
        )
