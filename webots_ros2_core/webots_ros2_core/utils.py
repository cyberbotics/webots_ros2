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

try:
    import webots_ros2_desktop.webots_path  # this module might not be installed
except ImportError:
    pass


def get_webots_home():
    """Path to the Webots installation directory."""
    webotsHome = None
    if 'ROS2_WEBOTS_HOME' in os.environ:
        webotsHome = os.environ['ROS2_WEBOTS_HOME']
    elif ('webots_ros2_desktop' in sys.modules and
          webots_ros2_desktop.webots_path and
          webots_ros2_desktop.webots_path.get_webots_home()):
        webotsHome = webots_ros2_desktop.webots_path.get_webots_home()
    elif 'WEBOTS_HOME' in os.environ:
        webotsHome = os.environ['WEBOTS_HOME']
    elif os.path.isdir('/usr/local/webots'):  # Linux default install
        webotsHome = '/usr/local/webots'
    elif os.path.isdir('/snap/webots/current/usr/share/webots'):  # Linux snap install
        webotsHome = '/snap/webots/current/usr/share/webots'
    elif os.path.isdir('C:\\Program Files\\Webots'):  # Windows default install
        webotsHome = 'C:\\Program Files\\Webots'
    elif os.path.isdir(os.getenv('LOCALAPPDATA') + '\\Programs\\Webots'):  # Windows user install
        webotsHome = os.getenv('LOCALAPPDATA') + '\\Programs\\Webots'
    else:
        sys.exit('Webots not found, you should either define "ROS2_WEBOTS_HOME", "WEBOTS_HOME" ' +
                 'or install the "webots_ros2_desktop" package.')
    os.environ['WEBOTS_HOME'] = webotsHome
    return webotsHome


def append_webots_lib_to_path():
    """Add the Webots 'lib' folder to the library path."""
    if sys.platform == 'linux':
        os.environ['LD_LIBRARY_PATH'] = (os.path.join(get_webots_home(), 'lib') + ':' +
                                         os.environ.get('LD_LIBRARY_PATH'))
    elif sys.platform == 'darwin':
        os.environ['DYLD_LIBRARY_PATH'] = (os.path.join(get_webots_home(), 'lib') + ':' +
                                           os.environ.get('DYLD_LIBRARY_PATH'))
    elif sys.platform == 'win32':
        os.environ['PATH'] = (os.path.join(get_webots_home(), 'msys64', 'mingw64', 'bin') + ';' +
                              os.environ.get('PATH'))
    else:
        sys.exit('Unsupported Platform!')


def append_webots_python_lib_to_path():
    """Add the Webots 'lib/pythonXY' folder to sys.path."""
    sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'python%d%d' %
                    (sys.version_info[0], sys.version_info[1])))


def get_webots_version():
    """Webots version as a string."""
    versionFile = os.path.join(get_webots_home(), 'resources', 'version.txt')
    with open(versionFile, "r") as f:
        return f.read().replace('\n', '').strip()
    return None
