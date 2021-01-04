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
import re
import sys
import argparse
import functools
import subprocess

from typing import List
from typing import Optional

from launch_ros.actions import Node
from launch.action import Action
from launch.launch_context import LaunchContext


MINIMUM_VERSION_STR = 'R2021a'
TARGET_VERSION_STR = 'R2021a'


@functools.total_ordering
class WebotsVersion:
    def __init__(self, version):
        self.version = version

        # Parse
        parts = re.findall(r'R(\d*)([a-b]{1})(\s((revision)|(rev))\s(\d){1}){0,1}', version)
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
        """Converts the version to a number that can be compared."""
        return self.year * 1e6 + (ord(self.release) - ord('a')) * 1e3 + self.revision

    def __str__(self):
        return self.version

    def short(self):
        return self.version.replace('revision ', 'rev').replace(' ', '-')


def get_webots_home(target_version=None, minimum_version=None, show_warning=False):
    # Normalize Webots version
    if target_version is not None and isinstance(target_version, str):
        target_version = WebotsVersion(target_version)
    if minimum_version is not None and isinstance(minimum_version, str):
        minimum_version = WebotsVersion(minimum_version)
    if target_version is None:
        target_version = WebotsVersion.target()
    if minimum_version is None:
        minimum_version = WebotsVersion.minimum()

    # Search target
    path = __get_webots_home(target_version, condition='eq')
    if path is not None:
        return path

    # Fallback to minumum
    path = __get_webots_home(target_version, condition='ge')
    if path is not None:
        found_version = WebotsVersion.from_path(path)
        if show_warning:
            print(f'WARNING: Target Webots version `{target_version}` is not found, fallback to `{found_version}`')
        return path

    return None


def __get_webots_home(target_version, condition='ge'):
    """Path to the Webots installation directory."""

    def version_condition(found, target):
        if target is None:
            return True
        if condition == 'eq':
            return found == target
        elif condition == 'ge':
            return found >= target
        raise Exception('`condition` can be `eq` or `ge`')

    # Search in the environment variables
    environment_variables = ['ROS2_WEBOTS_HOME', 'WEBOTS_HOME']
    for variable in environment_variables:
        if variable in os.environ and version_condition(WebotsVersion.from_path(os.environ[variable]), target_version):
            os.environ['WEBOTS_HOME'] = os.environ[variable]
            return os.environ[variable]

    # Use the 'which' command
    try:
        path = os.path.split(os.path.abspath(subprocess.check_output(['which', 'webots'])))[0]
        path = path.decode('utf-8')
        if os.path.isdir(path) and version_condition(WebotsVersion.from_path(path), target_version):
            os.environ['WEBOTS_HOME'] = path
            return path
    except subprocess.CalledProcessError:
        # The `which` command not available or Webots not found
        pass

    # Look at standard installation pathes
    paths = [
        '/usr/local/webots',                                    # Linux default install
        '/snap/webots/current/usr/share/webots',                # Linux snap install
        '/Applications/Webots.app',                             # macOS default install
        'C:\\Program Files\\Webots',                            # Windows default install
        os.getenv('LOCALAPPDATA', '') + '\\Programs\\Webots'    # Windows user install
    ]
    if target_version is not None:
        paths.append(os.path.join(os.environ['HOME'], '.ros', 'webots' + target_version.short(), 'webots'))
    for path in paths:
        if os.path.isdir(path) and version_condition(WebotsVersion.from_path(path), target_version):
            os.environ['WEBOTS_HOME'] = path
            return path
    return None


def append_webots_lib_to_path():
    """Add the Webots 'lib' folder to the library path."""
    webots_home = get_webots_home()
    if webots_home is None:
        print('Can\'t load Webots Python API, because Webots is not found.', file=sys.stderr)
        return False
    if sys.platform == 'linux':
        os.environ['LD_LIBRARY_PATH'] = os.path.join(webots_home, 'lib', 'controller') + ':' + os.environ.get('LD_LIBRARY_PATH')
        return True
    elif sys.platform == 'darwin':
        os.environ['DYLD_LIBRARY_PATH'] = os.path.join(
            webots_home, 'lib', 'controller') + ':' + os.environ.get('DYLD_LIBRARY_PATH')
        return True
    elif sys.platform == 'win32':
        os.environ['PATH'] = os.path.join(webots_home, 'lib', 'controller') + ';' + os.environ.get('PATH')
        return True
    else:
        print('Unsupported Platform!', file=sys.stderr)
        return False


def append_webots_python_lib_to_path():
    """Add the Webots 'lib/pythonXY' folder to sys.path."""
    if 'WEBOTS_HOME' not in os.environ:
        print('Can\'t load Webots Python API, because Webots is not found.', file=sys.stderr)
        return False

    if sys.platform == 'darwin':
        sys.path.append(
            os.path.join(
                os.environ['WEBOTS_HOME'],
                'lib',
                'controller',
                'python%d%d_brew' % (sys.version_info[0], sys.version_info[1])
            )
        )
        return True
    elif 'WEBOTS_HOME' in os.environ:
        sys.path.append(
            os.path.join(
                os.environ['WEBOTS_HOME'],
                'lib',
                'controller',
                'python%d%d' % (sys.version_info[0], sys.version_info[1])
            )
        )
        return True
    else:
        print('No Webots installation has been found!', file=sys.stderr)
        return False


def get_node_name_from_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--webots-node-name', dest='webots_node_name', default='webots_driver', help='Name of your drive node')
    args, _ = parser.parse_known_args()
    return args.webots_node_name


class ControllerLauncher(Node):
    """Utility class to launch a controller from a launch file."""

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        append_webots_lib_to_path()
        return super().execute(context)
