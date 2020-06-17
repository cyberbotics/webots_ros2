#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""This launcher simply start Webots."""

import os
import sys
from launch.actions import ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.substitution import Substitution
from webots_ros2_core.utils import get_webots_home


class _WebotsCommandSubstitution(Substitution):
    def __init__(self, *, world, gui, mode):
        self.__gui = gui if isinstance(gui, Substitution) else TextSubstitution(text=str(gui))
        self.__mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)
        self.__world = world if isinstance(world, Substitution) else TextSubstitution(text=world)

    def perform(self, context):
        # Add `webots` executable to command
        webots_path = get_webots_home()
        if sys.platform == 'win32':
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin')
        command = [os.path.join(webots_path, 'webots')]

        # Add `world`
        command += [context.perform_substitution(self.__world)]

        # Add parameters to hide GUI if needed
        if context.perform_substitution(self.__gui).lower() in ['false', '0']:
            command += [
                '--stdout',
                '--stderr',
                '--batch',
                '--no-sandbox',
                '--minimize'
            ]

        # Add mode
        command.append('--mode=' + context.perform_substitution(self.__mode))
        return ' '.join(command)


class WebotsLauncher(ExecuteProcess):
    def __init__(self, output='screen', world=None, gui=True, mode='realtime', **kwargs):
        command = _WebotsCommandSubstitution(
            world=world,
            gui=gui,
            mode=mode
        )

        super().__init__(
            output=output,
            cmd=[command],
            shell=True,
            **kwargs
        )
