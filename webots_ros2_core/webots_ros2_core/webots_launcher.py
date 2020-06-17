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
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext
from webots_ros2_core.utils import get_webots_home


class WebotsLauncher(ExecuteProcess):
    def __init__(self, output='screen', **kwargs):
        world = LaunchConfiguration('world')
        no_gui = LaunchConfiguration('no_gui', default=False)
        mode = LaunchConfiguration('mode', default='realtime')

        context = LaunchContext()

        # Add `webots` executable to command
        webots_path = get_webots_home()
        if sys.platform == 'win32':
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin')
        command = [os.path.join(webots_path, 'webots')]

        # Add `world`
        command += [world]

        # Add parameters to hide GUI if needed
        if no_gui.perform(context).lower() in ['true', '1']:
            command += [
                '--stdout',
                '--stderr',
                '--batch',
                '--no-sandbox',
                '--minimize'
            ]

        # Add mode
        command.append('--mode=' + mode.perform(context))

        super().__init__(
            cmd=command,
            output=output,
            **kwargs
        )
