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

from urdf2webots.importer import convert2urdf

from ament_index_python.packages import get_package_share_directory


def main(args=None, input=None):
    parser = argparse.ArgumentParser(usage='usage: %prog --input=my_robot.urdf [options]')
    parser.add_argument('--input', dest='inFile', default='',
                        help='Specifies the urdf file to convert.')
    parser.add_argument('--output', dest='outFile', default='',
                        help='Specifies the name of the resulting PROTO file.')
    parser.add_argument('--normal', dest='normal', action='store_true', default=False,
                        help='If set, the normals are exported if defined.')
    parser.add_argument('--box-collision', dest='boxCollision',
                        action='store_true', default=False,
                        help='If set, the bounding objects are approximated using boxes.')
    parser.add_argument('--disable-mesh-optimization', dest='disableMeshOptimization',
                        action='store_true', default=False,
                        help='If set, the duplicated vertices are not removed from the meshes.')
    # use 'parse_known_args' because ROS2 adds a lot of internal arguments
    arguments, unknown = parser.parse_known_args()
    file = os.path.abspath(input) if input is not None else os.path.abspath(arguments.inFile)
    if not file:
        sys.exit('Input file not specified (should be specified with the "--input" argument).')
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
    convert2urdf(inFile=urdfFile,
                 outFile=arguments.outFile,
                 normal=arguments.normal,
                 boxCollision=arguments.boxCollision,
                 disableMeshOptimization=arguments.disableMeshOptimization)
    # remove temporary file
    if generatedFile:
        os.remove(urdfFile)


if __name__ == '__main__':
    main()
