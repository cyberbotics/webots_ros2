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

"""Convert an URDF File into a PROTO."""

import argparse
import os
import re
import sys
import tempfile

import time

from ament_index_python.packages import get_package_share_directory


def main(args=None):
    parser = argparse.ArgumentParser(usage='usage: %prog --input=my_robot.urdf [options]')
    parser.add_argument('--input', dest='inFile', default='',
                        help='Specifies the urdf file to convert.')
    # use 'parse_known_args' because ROS2 adds a lot of internal arguments
    arguments, unknown = parser.parse_known_args()
    file = os.path.abspath(arguments.inFile)
    if not os.path.isfile(file):
        sys.exit('"%s" file does not exist.' % arguments.inFile)
    elif not file.endswith('.urdf'):
        sys.exit('"%s" is not an urdf file.' % file)
    content = ''
    with open(file, 'r') as f:
        content = f.read()
    # look for package-relative file path and replace them
    urdfFile = file
    generatedFile = False
    packages = re.findall('filename="package:\/\/([^\/]*)', content)
    for package in set(packages):  # do the replacement
        packagePath = ''
        try:
            packagePath = get_package_share_directory(package)
        except LookupError:
            sys.exit('This urdf depends on the "%s" package, but this package could not be found' %
                     package)
        content = content.replace('package://%s' % package, '%s' % packagePath)
    if packages:  # some package-relative file paths have been found
        # generate a temporary file with the replacement content
        urdfFile = tempfile.mkstemp(suffix='.urdf')[1]
        generatedFile = True
        with open(urdfFile, 'w') as f:
            f.write(content)
    #TODO: call importer
    # remove temporary file
    if generatedFile:
        os.remove(urdfFile)


if __name__ == '__main__':
    main()
