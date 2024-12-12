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

"""Utilitary functions for the package."""

import os
import re
import sys
import shutil
import tarfile
import functools
import subprocess
import urllib.request
from pathlib import Path
from platform import uname


# The minimal version should be the last stable release of Webots (both on master and develop branches)
MINIMUM_VERSION_STR = 'R2025a'


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
    def minimum():
        return WebotsVersion(MINIMUM_VERSION_STR)

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


def is_wsl():
    return 'microsoft-standard' in uname().release


def get_wsl_ip_address():
    try:
        file = open('/etc/resolv.conf', 'r')
    except IOError:
        # /etc/resolv.conf doesn't exist, can't be read, etc.
        # Use the default resolver configuration.
        return '127.0.0.1'
    try:
        for line in file:
            if len(line) == 0 or line[0] == '#' or line[0] == ';':
                continue
            tokens = line.split()
            if len(tokens) == 0:
                continue
            if tokens[0] == 'nameserver':
                file.close()
                if len(tokens[1]) == 0:
                    return '127.0.0.1'
                return tokens[1]
    finally:
        file.close()


def has_shared_folder():
    return 'WEBOTS_SHARED_FOLDER' in os.environ


def host_shared_folder():
    shared_folder_list = os.environ['WEBOTS_SHARED_FOLDER'].split(':')
    return shared_folder_list[0]


def container_shared_folder():
    shared_folder_list = os.environ['WEBOTS_SHARED_FOLDER'].split(':')
    return shared_folder_list[1]


def is_docker():
    mountinfo = Path("/proc/self/mountinfo")
    return mountinfo.is_file() and "docker" in mountinfo.read_text()


def get_host_ip():
    if is_docker():
        return "host.docker.internal"

    try:
        output = subprocess.run(['ip', 'route'], check=True, stdout=subprocess.PIPE, universal_newlines=True)
        for line in output.stdout.split('\n'):
            fields = line.split()
            if fields and fields[0] == 'default':
                return fields[2]
        sys.exit('Unable to get host IP address.')
    except subprocess.CalledProcessError:
        sys.exit('Unable to get host IP address. \'ip route\' could not be executed.')


def controller_protocol():
    protocol = 'tcp' if (has_shared_folder() or is_wsl()) else 'ipc'
    return protocol


def controller_ip_address():
    ip_address = get_host_ip() if has_shared_folder() else get_wsl_ip_address()
    return ip_address


def controller_url_prefix(port='1234'):
    if has_shared_folder() or is_wsl():
        return 'tcp://' + (get_host_ip() if has_shared_folder() else get_wsl_ip_address()) + ':' + port + '/'
    else:
        return ''


def get_webots_home(show_warning=False):
    def version_min(found, minimum):
        if minimum is None:
            return True
        if found is None:
            return False
        return found >= minimum

    # Search in the environment variables
    environment_variables = ['ROS2_WEBOTS_HOME', 'WEBOTS_HOME']
    for variable in environment_variables:
        if variable in os.environ and os.path.isdir(os.environ[variable]) and WebotsVersion.from_path(os.environ[variable]):
            os.environ['WEBOTS_HOME'] = os.environ[variable]
            return os.environ[variable]
        elif variable in os.environ and \
                (not os.path.isdir(os.environ[variable]) or WebotsVersion.from_path(os.environ[variable]) is None):
            print(f'WARNING: Webots directory `{os.environ[variable]}` specified in `{variable}` is not a valid Webots '
                  'directory or is not found.')

    # Normalize Webots version
    minimum_version = WebotsVersion.minimum()

    # Look at standard installation pathes
    if is_wsl():
        paths = [
            '/mnt/c/Program Files/Webots'                           # WSL default Windows install
        ]
    elif sys.platform == 'linux':
        paths = [
            '/usr/local/webots',                                    # Linux default install
            '/snap/webots/current/usr/share/webots'                 # Linux snap install
        ]
    elif sys.platform == 'darwin':
        paths = [
            '/Applications/Webots.app'                              # macOS default install
        ]
    elif sys.platform == 'win32':
        paths = [
            'C:\\Program Files\\Webots'                             # Windows default install
        ]
    # Add automatic installation path to pathes list
    paths.append(os.path.join(str(Path.home()), '.ros', 'webots' + minimum_version.short(), 'webots'))

    # Check if default pathes contain target version of Webots
    for path in paths:
        if os.path.isdir(path) and version_min(WebotsVersion.from_path(path), minimum_version):
            os.environ['WEBOTS_HOME'] = path
            if show_warning:
                print('WARNING: No valid Webots directory specified in `ROS2_WEBOTS_HOME` and `WEBOTS_HOME`, fallback to '
                      f'default installation folder {path}.')
            return path

    return None


def __install_webots(installation_directory):
    minimum_version = WebotsVersion.minimum()

    def on_download_progress_changed(count, block_size, total_size):
        percent = count * block_size * 100 / total_size
        sys.stdout.write(f'\rDownloading... {percent:.2f}%')
        sys.stdout.flush()

    print(f'Installing Webots {minimum_version}... This might take some time.')

    # Remove previous archive
    if is_wsl():
        archive_name = f'webots-{minimum_version.short()}_setup.exe'
        archive_path = os.path.join('/mnt/c/Temp', archive_name)
    else:
        installation_path = os.path.abspath(os.path.join(installation_directory, 'webots'))
        archive_name = f'webots-{minimum_version.short()}-x86-64.tar.bz2'
        archive_path = os.path.join(installation_directory, archive_name)

        # Remove previous webots folder
        if os.path.exists(installation_path):
            shutil.rmtree(installation_path)

    # Get Webots archive
    if not os.path.exists(archive_path):
        url = f'https://github.com/cyberbotics/webots/releases/download/{minimum_version.short()}/'
        urllib.request.urlretrieve(url + archive_name, archive_path, reporthook=on_download_progress_changed)
        print('')
    else:
        print(f'Using installation present at `{archive_path}`...')

    # Extract Webots archive
    if is_wsl():
        print('Installing...')
        subprocess.check_output(f'{archive_path} /SILENT /ALLUSERS', shell=True)
        os.remove(archive_path)
        os.environ['WEBOTS_HOME'] = '/mnt/c/Program Files/Webots'
    else:
        installation_subdirectory = os.path.join(installation_directory, 'webots' + minimum_version.short())
        print('Extracting...')
        tar = tarfile.open(archive_path, 'r:bz2')
        tar.extractall(installation_subdirectory)
        tar.close()
        os.remove(archive_path)
        os.environ['WEBOTS_HOME'] = os.path.join(installation_path, 'webots')


def handle_webots_installation():
    minimum_version = WebotsVersion.minimum()
    if is_wsl() or sys.platform == 'win32':
        installation_directory = 'C:\\Program Files\\Webots'
    else:
        installation_directory = os.path.join(str(Path.home()), '.ros')
    webots_release_url = f'https://github.com/cyberbotics/webots/releases/tag/{minimum_version.short()}'

    print(
        f'Webots {minimum_version} was not found in your system.\n'
        f'- If you want to manually install Webots {minimum_version} please download '
        f'it from {webots_release_url}.\n'
        f'- If you already have Webots {minimum_version} installed please then specify the '
        f'`WEBOTS_HOME` environment variable.\n'
    )

    method = input(
        f'Do you want Webots {minimum_version} to be automatically installed in {installation_directory} ([Y]es/[N]o)?: ')

    if method.lower() == 'y':
        __install_webots(installation_directory)
        webots_path = get_webots_home()
        if webots_path is None:
            sys.exit(f'Failed to install Webots {minimum_version}')
    else:
        sys.exit(f'Missing Webots version {minimum_version}')
