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

import argparse
import os
import subprocess
import sys

from webots_ros2_core.utils import get_webots_home


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', dest='world', default='', help='Path to the world to load.')
    parser.add_argument('--mode', dest='mode', default='realtime', help='Startup mode.')
    parser.add_argument('--no-gui', dest='noGui', action='store_true',
                        help='Start Webots with minimal GUI.')
    args = parser.parse_args()
    webotsPath = get_webots_home()
    if sys.platform == 'win32':
        webotsPath = os.path.join(webotsPath, 'msys64', 'mingw64', 'bin')
    command = [os.path.join(webotsPath, 'webots'), '--mode=' + args.mode, args.world]
    if args.noGui:
        command.append('--stdout')
        command.append('--stderr')
        command.append('--batch')
        command.append('--no-sandbox')
        command.append('--minimize')

    subprocess.call(command)


if __name__ == '__main__':
    main()
