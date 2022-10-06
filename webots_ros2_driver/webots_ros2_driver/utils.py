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
import sys
import shutil
import tarfile
import functools
import subprocess
import urllib.request
from pathlib import Path


TARGET_VERSION_STR = 'R2022b'


@functools.total_ordering
class WebotsVersion:
    def __init__(self, version):
        self.version = version

        # Parse
        parts = re.findall(r'R(\d*)([a-d]{1})(\s((revision)|(rev))\s(\d){1}){0,1}', version)
        self.year = int(parts[0][0])
        self.release = parts[0][1].lower()
        self.revision = 0
        if parts[0][-1] != '':
            self.revision = int(parts[0][-1])

    @staticmethod
    def from_path(path):
        version_file = os.path.join(path, 'resources', 'version.txt')
        if not os.path.isfile(version_file):
            return None
        with open(version_file, 'r') as f:
            return WebotsVersion(f.read().strip())
        return None

    @staticmethod
    def target():
        return WebotsVersion(TARGET_VERSION_STR)

    def __eq__(self, other):
        if other.get_number() == self.get_number():
            return True
        return False

    def __ne__(self, other):
        return not self == other

    def __gt__(self, other):
        if other.get_number() < self.get_number():
            return True
        return False

    def get_number(self):
        """Convert the version to a number that can be compared."""
        return self.year * 1e6 + (ord(self.release) - ord('a')) * 1e3 + self.revision

    def __str__(self):
        return self.version

    def short(self):
        return self.version.replace('revision ', 'rev').replace(' ', '-')


def get_webots_home(isWSL, show_warning=False):
    def version_equal(found, target):
        if target is None:
            return True
        if found is None:
            return False
        return found == target

    # Search in the environment variables
    environment_variables = ['ROS2_WEBOTS_HOME', 'WEBOTS_HOME']
    for variable in environment_variables:
        if variable in os.environ and os.path.isdir(os.environ[variable]) and WebotsVersion.from_path(os.environ[variable]):
            os.environ['WEBOTS_HOME'] = os.environ[variable]
            return os.environ[variable]
        elif variable in os.environ and (not os.path.isdir(os.environ[variable]) or WebotsVersion.from_path(os.environ[variable]) is None):
            print(f'WARNING: Webots directory `{os.environ[variable]}` specified in `{variable}` is not a valid Webots directory or is not found.')

    # Normalize Webots version
    target_version = WebotsVersion.target()

    # Look at standard installation pathes
    if isWSL:
        paths = [
            '/mnt/c/Program Files/Webots'                           # WSL default Windows install
        ]
    elif sys.platform == 'linux':
        paths = [
            '/usr/local/webots',                                    # Linux default install
            '/snap/webots/current/usr/share/webots',                # Linux snap install
        ]
    elif sys.platform == 'darwin':
        paths = [
            '/Applications/Webots.app',                             # macOS default install
        ]
    # Add automatic installation path to pathes list
    paths.append(os.path.join(str(Path.home()), '.ros', 'webots' + target_version.short(), 'webots'))

    # Check if default pathes contain target version of Webots
    for path in paths:
        if os.path.isdir(path) and version_equal(WebotsVersion.from_path(path), target_version):
            os.environ['WEBOTS_HOME'] = path
            if show_warning:
                print(f'WARNING: No valid Webots directory specified in `ROS2_WEBOTS_HOME` and `WEBOTS_HOME`, fallback to default installation folder {path}.')
            return path

    return None

def __install_webots(installation_directory, isWSL):
    target_version = WebotsVersion.target()

    def on_download_progress_changed(count, block_size, total_size):
        percent = count * block_size * 100 / total_size
        sys.stdout.write(f'\rDownloading... {percent:.2f}%')
        sys.stdout.flush()

    print(f'Installing Webots {target_version}... This might take some time.')

    # Remove previous archive
    if isWSL:
        archive_name = f'webots-{target_version.short()}_setup.exe'
        archive_path = os.path.join('/mnt/c/Temp', archive_name)
    else:
        installation_path = os.path.abspath(os.path.join(installation_directory, 'webots'))
        archive_name = f'webots-{target_version.short()}-x86-64.tar.bz2'
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
    if isWSL:
        print('Installing...')
        subprocess.check_output(f'{archive_path} /SILENT /ALLUSERS', shell=True)
        os.remove(archive_path)
        os.environ['WEBOTS_HOME'] = '/mnt/c/Program Files/Webots'
    else:
        installation_subdirectory = os.path.join(installation_directory, 'webots' + target_version.short())
        print('Extracting...')
        tar = tarfile.open(archive_path, 'r:bz2')
        tar.extractall(installation_subdirectory)
        tar.close()
        os.remove(archive_path)
        os.environ['WEBOTS_HOME'] = os.path.join(installation_path, 'webots')

def handle_webots_installation(isWSL):
    target_version = WebotsVersion.target()
    installation_directory = f'C:\\Program Files\\Webots' if isWSL else os.path.join(str(Path.home()), '.ros')
    webots_release_url = f'https://github.com/cyberbotics/webots/releases/tag/{target_version.short()}'

    print(
        f'Webots {target_version} was not found in your system.\n'
        f'- If you want to manually install Webots {target_version} please download '
        f'it from {webots_release_url}.\n'
        f'- If you already have Webots {target_version} installed please then specify the '
        f'`WEBOTS_HOME` environment variable.\n'
    )

    location_text = f'in `{installation_directory}` '
    method = input(
        f'Do you want Webots {target_version} to be automatically installed {location_text}([Y]es/[N]o)?: ')

    if method.lower() == 'y':
        __install_webots(installation_directory, isWSL)
        webots_path = get_webots_home(isWSL)
        if webots_path is None:
            sys.exit(f'Failed to install Webots {target_version}')
    else:
        sys.exit(f'Missing Webots version {target_version}')
