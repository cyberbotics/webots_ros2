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

from webots_ros2_driver.utils import (get_webots_home,
                                      handle_webots_installation,
                                      is_wsl,
                                      has_shared_folder,
                                      container_shared_folder,
                                      controller_url_prefix)


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
    def __init__(self, output='screen', world=None, gui=True, mode='realtime', stream=False, ros2_supervisor=False,
                 port='1234', **kwargs):
        if sys.platform == 'win32':
            print('WARNING: Native webots_ros2 compatibility with Windows is deprecated and will be removed soon. Please use a '
                  'WSL (Windows Subsystem for Linux) environment instead.', file=sys.stderr)
            print('WARNING: Check https://github.com/cyberbotics/webots_ros2/wiki/Complete-Installation-Guide for more '
                  'information.', file=sys.stderr)
        self.__is_wsl = is_wsl()
        self.__has_shared_folder = has_shared_folder()
        self.__is_supervisor = ros2_supervisor
        if self.__is_supervisor:
            self._supervisor = Ros2SupervisorLauncher(port=port)

        # Find Webots executable
        if not self.__has_shared_folder:
            webots_path = get_webots_home(show_warning=True)
            if webots_path is None:
                handle_webots_installation()
                webots_path = get_webots_home()
            if self.__is_wsl:
                webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin', 'webots.exe')
            else:
                webots_path = os.path.join(webots_path, 'webots')
        else:
            webots_path = ''

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
        if isinstance(stream, bool):
            stream_argument = _ConditionalSubstitution(condition=stream, true_value='--stream')
        else:
            stream_argument = "--stream=" + stream
        port_argument = '--port=' + port

        xvfb_run_prefix = []

        if 'WEBOTS_OFFSCREEN' in os.environ:
            xvfb_run_prefix.append('xvfb-run')
            xvfb_run_prefix.append('--auto-servernum')
            no_rendering = '--no-rendering'

        # Initialize command to start Webots remotely through TCP
        if self.__has_shared_folder:
            webots_tcp_client = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts',
                                 'webots_tcp_client.py'))
            super().__init__(
                output=output,
                cmd=[
                    'python3',
                    webots_tcp_client,
                    stream_argument,
                    port_argument,
                    no_rendering,
                    stdout,
                    stderr,
                    minimize,
                    '--batch',
                    ['--mode=', mode],
                    os.path.basename(self.__world_copy.name),
                ],
                name='webots_tcp_client',
                **kwargs
            )
        # Initialize command to start Webots locally
        else:
            # no_rendering, stdout, stderr, minimize
            super().__init__(
                output=output,
                cmd=xvfb_run_prefix + [
                    webots_path,
                    stream_argument,
                    port_argument,
                    no_rendering,
                    stdout,
                    stderr,
                    minimize,
                    world,
                    '--batch',
                    ['--mode=', mode],
                ],
                name='webots',
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

        # look for a wbproj file and copy if available
        wbproj_path = Path(world_path).with_name('.' + Path(world_path).stem + '.wbproj')

        if wbproj_path.exists():
            wbproj_copy_path = Path(self.__world_copy.name).with_name('.' + Path(self.__world_copy.name).stem +
                                                                      '.wbproj')
            shutil.copy2(wbproj_path, wbproj_copy_path)

        # copy sumo network file if it exists
        sumonet_path = Path(world_path).with_name(Path(world_path).stem + '_net')
        if sumonet_path.exists():
            sumonet_copy_path = Path(self.__world_copy.name).with_name(Path(self.__world_copy.name).stem + '_net')
            shutil.copytree(sumonet_path, sumonet_copy_path)

        # Copy forests
        forests_path = Path(world_path).with_name('forest')
        if forests_path.exists():
            forests_copy = Path(self.__world_copy.name).with_name('forest')

            if forests_copy.exists():
                shutil.rmtree(forests_copy)

            shutil.copytree(forests_path, forests_copy)

        # Update relative paths in the world
        with open(self.__world_copy.name, 'r') as file:
            content = file.read()

        for match in re.finditer('\"((?:[^\"]*)\\.(?:jpe?g|png|hdr|obj|stl|dae|wav|mp3|proto))\"', content):
            url_path = match.group(1)

            # Absolute path or Webots relative path or Web paths
            if os.path.isabs(url_path) or url_path.startswith('webots://') or url_path.startswith('http://') \
                    or url_path.startswith('https://'):
                continue

            new_url_path = os.path.split(world_path)[0] + '/' + url_path
            if self.__has_shared_folder:
                # Copy asset to shared folder
                shutil.copy(new_url_path, os.path.join(container_shared_folder(), os.path.basename(new_url_path)))
                new_url_path = './' + os.path.basename(new_url_path)
            if self.__is_wsl:
                command = ['wslpath', '-w', new_url_path]
                new_url_path = subprocess.check_output(command).strip().decode('utf-8').replace('\\', '/')
            new_url_path = '"' + new_url_path + '"'
            url_path = '"' + url_path + '"'
            content = content.replace(url_path, new_url_path)

        with open(self.__world_copy.name, 'w') as file:
            file.write(content)

        # Add the Ros2Supervisor
        if self.__is_supervisor:
            indent = '  '
            world_file = open(self.__world_copy.name, 'a')
            world_file.write('Robot {\n')
            world_file.write(indent + 'name "Ros2Supervisor"\n')
            world_file.write(indent + 'controller "<extern>"\n')
            world_file.write(indent + 'supervisor TRUE\n')
            world_file.write('}\n')
            world_file.close()

        # Copy world file to shared folder
        if self.__has_shared_folder:
            shared_world_file = os.path.join(container_shared_folder(), os.path.basename(self.__world_copy.name))
            shutil.copy(self.__world_copy.name, shared_world_file)
            if wbproj_path.exists():
                shared_wbproj_copy_path = Path(shared_world_file).with_name('.' + Path(shared_world_file).stem + '.wbproj')
                shutil.copy(wbproj_path, shared_wbproj_copy_path)

        # Execute process
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

        # Clean the content of the shared directory for next run
        if self.__has_shared_folder:
            for filename in os.listdir(container_shared_folder()):
                file_path = os.path.join(container_shared_folder(), filename)
                try:
                    if os.path.isfile(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as error:
                    print(f'Failed to delete {file_path}. Reason: {error}.')

        return super()._shutdown_process(context, send_sigint=send_sigint)


class Ros2SupervisorLauncher(Node):
    def __init__(self, output='screen', respawn=True, port='1234', **kwargs):
        # Launch the Ros2Supervisor node
        super().__init__(
            package='webots_ros2_driver',
            executable='ros2_supervisor.py',
            namespace='Ros2Supervisor',
            remappings=[('/Ros2Supervisor/clock', '/clock')],
            output=output,
            # Set WEBOTS_HOME to the webots_ros2_driver installation folder
            # to load the correct libController libraries from the Python API
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix(port) + 'Ros2Supervisor',
                            'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
            respawn=respawn,
            **kwargs
        )
