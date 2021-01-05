#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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
import subprocess
import urllib.request
from pathlib import Path
from launch.actions import ExecuteProcess
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from webots_ros2_core.utils import get_webots_home, WebotsVersion


class _WebotsCommandSubstitution(Substitution):
    def __init__(self, *, world, gui, mode):
        self.__gui = gui if isinstance(gui, Substitution) else TextSubstitution(text=str(gui))
        self.__mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)
        self.__world = world if isinstance(world, Substitution) else TextSubstitution(text=world)

    def __get_archive_name(self, version):
        if sys.platform == 'darwin' or sys.platform == 'linux':
            return f'webots-{version.short()}-x86-64.tar.bz2'
        return f'webots-{version.short()}_setup.exe'

    def __install_webots(self, installation_directory):
        target_version = WebotsVersion.target()

        def on_download_progress_changed(count, block_size, total_size):
            percent = count*block_size*100/total_size
            sys.stdout.write(f'\rDownloading... {percent:.2f}%')
            sys.stdout.flush()

        print(f'Installing Webots {target_version}... This might take some time.')

        # Remove previous archive
        installation_path = os.path.abspath(os.path.join(installation_directory, 'webots'))
        archive_name = self.__get_archive_name(target_version)
        archive_path = os.path.join(installation_directory, archive_name)

        # Remove previous webots folder
        if os.path.exists(installation_path):
            shutil.rmtree(installation_path)

        # Get Webots archive
        if not os.path.exists(archive_path):
            url = f'https://github.com/cyberbotics/webots/releases/download/{target_version.short()}/'
            urllib.request.urlretrieve(url + archive_name, archive_path, reporthook=on_download_progress_changed)
            print('')
        else:
            print(f'Using installation present at `{archive_path}`...')

        # Extract Webots archive
        installation_subdirectory = os.path.join(installation_directory, 'webots' + target_version.short())
        if sys.platform == 'darwin' or sys.platform == 'linux':
            print('Extracting...')
            tar = tarfile.open(archive_path, 'r:bz2')
            tar.extractall(installation_subdirectory)
            tar.close()
            os.remove(archive_path)
            os.environ['WEBOTS_HOME'] = os.path.join(installation_path, 'webots')
        else:
            print('Installing...')
            subprocess.check_output(f'{archive_path} /SILENT /CURRENTUSER', shell=True)

    def __handle_webots_installation(self):
        target_version = WebotsVersion.target()
        installation_directory = os.path.join(str(Path.home()), '.ros')
        webots_release_url = f'https://github.com/cyberbotics/webots/releases/tag/{target_version.short()}'

        print(
            f'Webots {target_version} was not found in your system.\n'
            f'- If you want to manually install Webots {target_version} please download '
            f'it from {webots_release_url}.\n'
            f'- If you already have installed Webots {target_version} installed please specify the '
            f'`WEBOTS_HOME` environment variable.\n'
        )

        location_text = '' if sys.platform == 'win32' else f'in `{installation_directory}` '
        method = input(
            f'Do you want Webots {target_version} to be automatically installed {location_text}([Y]es/[N]o)?: ')

        if method.lower() == 'y':
            self.__install_webots(installation_directory)
            webots_path = get_webots_home()
            if webots_path is None:
                sys.exit(f'Failed to install Webots {target_version}')
        else:
            sys.exit(f'Missing Webots version {target_version}')

    def perform(self, context):
        webots_path = get_webots_home(show_warning=True)
        if webots_path is None:
            self.__handle_webots_installation()
            webots_path = get_webots_home(show_warning=True)

        # Add `webots` executable to command
        if sys.platform == 'win32':
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin')
        command = [os.path.join(webots_path, 'webots')]

        # Add `world`
        command += [context.perform_substitution(self.__world)]

        # Hide welcome window
        command += ['--batch']

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
