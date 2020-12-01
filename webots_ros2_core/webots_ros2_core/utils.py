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
import re
import sys
import argparse
import subprocess

from typing import List
from typing import Optional

from launch_ros.actions import Node
from launch.action import Action
from launch.launch_context import LaunchContext


def get_required_webots_version():
    """Return the Webots version compatible with this version of the package."""
    return 'R2020b revision 1'


def make_short_version(version):
    """Transform a long Webots version in a short one."""
    return version.replace('revision ', 'rev').replace(' ', '-')


def get_required_webots_version_short():
    """Return the Webots short version compatible with this version of the package."""
    return make_short_version(get_required_webots_version())


def get_webots_home(version=get_required_webots_version()):
    """Path to the Webots installation directory."""
    # search first with the environment variables
    environVariables = ['ROS2_WEBOTS_HOME', 'WEBOTS_HOME']
    for variable in environVariables:
        if variable in os.environ and os.path.isdir(os.environ[variable]) and \
                get_webots_version(os.environ[variable]) == version:
            os.environ['WEBOTS_HOME'] = os.environ[variable]
            return os.environ[variable]
    # then using the 'which' command
    try:
        path = os.path.split(os.path.abspath(subprocess.check_output(['which', 'webots'])))[0]
        path = path.decode('utf-8')
        if os.path.isdir(path) and get_webots_version(path) == version:
            os.environ['WEBOTS_HOME'] = path
            return path
    except subprocess.CalledProcessError:
        pass  # which not available or Webots not found
    # finally, look at standard installation pathes
    pathes = [
        os.path.join(os.environ['HOME'], '.ros', 'webots' + make_short_version(version), 'webots'),
        '/usr/local/webots',  # Linux default install
        '/snap/webots/current/usr/share/webots',  # Linux snap install
        '/Applications/Webots.app',  # macOS default install
        'C:\\Program Files\\Webots',  # Windows default install
        os.getenv('LOCALAPPDATA', '') + '\\Programs\\Webots'  # Windows user install
    ]
    for path in pathes:
        if os.path.isdir(path) and get_webots_version(path) == version:
            os.environ['WEBOTS_HOME'] = path
            return path
    return None


def append_webots_lib_to_path():
    """Add the Webots 'lib' folder to the library path."""
    webotsHome = get_webots_home()
    if webotsHome is None:
        print('Can\'t load Webots Python API, because Webots is not found.', file=sys.stderr)
        return False
    if sys.platform == 'linux':
        if get_webots_version_major_number() <= 2019:
            os.environ['LD_LIBRARY_PATH'] = (os.path.join(webotsHome, 'lib') + ':'
                                             + os.environ.get('LD_LIBRARY_PATH'))
        else:
            os.environ['LD_LIBRARY_PATH'] = (os.path.join(webotsHome, 'lib', 'controller')
                                             + ':' + os.environ.get('LD_LIBRARY_PATH'))
        return True
    elif sys.platform == 'darwin':
        if get_webots_version_major_number() <= 2019:
            os.environ['DYLD_LIBRARY_PATH'] = (os.path.join(webotsHome, 'lib') + ':' +
                                               os.environ.get('DYLD_LIBRARY_PATH'))
        else:
            os.environ['DYLD_LIBRARY_PATH'] = (os.path.join(webotsHome, 'lib', 'controller')
                                               + ':' + os.environ.get('DYLD_LIBRARY_PATH'))
        return True
    elif sys.platform == 'win32':
        if get_webots_version_major_number() <= 2019:
            os.environ['PATH'] = (os.path.join(webotsHome, 'msys64', 'mingw64', 'bin')
                                  + ';' + os.environ.get('PATH'))
        else:
            os.environ['PATH'] = (os.path.join(webotsHome, 'lib', 'controller') + ';' +
                                  os.environ.get('PATH'))
        return True
    else:
        print('Unsupported Platform!', file=sys.stderr)
        return False


def append_webots_python_lib_to_path():
    """Add the Webots 'lib/pythonXY' folder to sys.path."""
    if 'WEBOTS_HOME' not in os.environ:
        print('Can\'t load Webots Python API, because Webots is not found.', file=sys.stderr)
        return False
    if get_webots_version_major_number() <= 2019:
        sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'python%d%d' %
                                     (sys.version_info[0], sys.version_info[1])))
        return True
    elif sys.platform == 'darwin':
        sys.path.append(os.path.join(os.environ['WEBOTS_HOME'],
                                     'lib',
                                     'controller',
                                     'python%d%d_brew' % (sys.version_info[0], sys.version_info[1])))
        return True
    elif 'WEBOTS_HOME' in os.environ:
        sys.path.append(os.path.join(os.environ['WEBOTS_HOME'],
                                     'lib',
                                     'controller',
                                     'python%d%d' % (sys.version_info[0], sys.version_info[1])))
        return True
    else:
        print('No Webots installation has been found!', file=sys.stderr)
        return False


def get_webots_version_major_number():
    """Webots major version as an integer."""
    versionString = get_webots_version()
    if versionString is None:
        return 0
    match = re.match(r'R(\d*).*', versionString)
    if match:
        return int(match.groups()[0])
    return 0


def get_webots_version(path=None):
    """Webots version as a string."""
    if path is None:
        path = get_webots_home()
    if path is None:
        return None
    versionFile = os.path.join(path, 'resources', 'version.txt')
    if not os.path.isfile(versionFile):
        return None
    with open(versionFile, 'r') as f:
        return f.read().replace('\n', '').strip()
    return None


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
