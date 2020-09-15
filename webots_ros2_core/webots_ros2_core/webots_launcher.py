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
import shutil
import sys
import tarfile
import urllib.request
from launch.actions import ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.substitution import Substitution
from webots_ros2_core.utils import get_webots_home, get_webots_version

WEBOTS_VERSION = 'R2020b revision 1'


class _WebotsCommandSubstitution(Substitution):
    def __init__(self, *, world, gui, mode):
        self.__gui = gui if isinstance(gui, Substitution) else TextSubstitution(text=str(gui))
        self.__mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)
        self.__world = world if isinstance(world, Substitution) else TextSubstitution(text=world)

    def install_webots(self):
        # Remove previous archive
        webotsShortVersion = WEBOTS_VERSION.replace('revision ', 'rev').replace(' ', '-')
        installationDirectory = os.path.join(os.environ['HOME'], '.ros')
        installationPath = os.path.abspath(os.path.join(installationDirectory, 'webots'))
        archiveName = 'webots-%s-x86-64.tar.bz2' % webotsShortVersion
        archivePath = os.path.join(installationDirectory, archiveName)
        if os.path.exists(archivePath):
            os.remove(archivePath)
        # Remove previous webots folder
        if os.path.exists(installationPath):
            shutil.rmtree(installationPath)
        # Get Webots archive
        print('\033[33mInstalling Webots (%s), this might take some time.\033[0m' % WEBOTS_VERSION)
        url = 'https://github.com/cyberbotics/webots/releases/download/%s/' % webotsShortVersion
        urllib.request.urlretrieve(url + archiveName, archivePath)
        # Extract Webots archive
        tar = tarfile.open(archiveName, 'r:bz2')
        tar.extractall()
        tar.close()
        os.environ['WEBOTS_HOME'] = installationPath

    def perform(self, context):
        webots_path = get_webots_home()
        # check Webots version
        version = get_webots_version(webots_path)
        if version != WEBOTS_VERSION:
            self.install_webots()
        # Add `webots` executable to command
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
