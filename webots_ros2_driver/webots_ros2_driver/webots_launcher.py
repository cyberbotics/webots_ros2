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
import sys
from launch.actions import ExecuteProcess
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from webots_ros2_driver.utils import get_webots_home, handle_webots_installation


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
        # Find Webots executable
        webots_path = get_webots_home(show_warning=True)
        if webots_path is None:
            handle_webots_installation()
            webots_path = get_webots_home()
        if sys.platform == 'win32':
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin')
        webots_path = os.path.join(webots_path, 'webots')

        mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)
        world = world if isinstance(world, Substitution) else TextSubstitution(text=world)

        no_rendering = _ConditionalSubstitution(condition=gui, false_value='--no-rendering')
        stdout = _ConditionalSubstitution(condition=gui, false_value='--stdout')
        stderr = _ConditionalSubstitution(condition=gui, false_value='--stderr')
        no_sandbox = _ConditionalSubstitution(condition=gui, false_value='--no-sandbox')
        if sys.platform == 'win32':
            # Windows doesn't have the sandbox argument
            no_sandbox = ''
        minimize = _ConditionalSubstitution(condition=gui, false_value='--minimize')
        stream_argument = _ConditionalSubstitution(condition=stream, true_value='--stream')

        xvfb_run_prefix = []
        if 'WEBOTS_OFFSCREEN' in os.environ:
            xvfb_run_prefix.append('xvfb-run')
            xvfb_run_prefix.append('--auto-servernum')

        # no_rendering, stdout, stderr, no_sandbox, minimize
        super().__init__(
            output=output,
            cmd=xvfb_run_prefix + [
                webots_path,
                stream_argument,
                no_rendering,
                stdout,
                stderr,
                no_sandbox,
                minimize,
                world,
                '--batch',
                ['--mode=', mode],
            ],
            **kwargs
        )
