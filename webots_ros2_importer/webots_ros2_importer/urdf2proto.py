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

"""Convert an URDF File into a PROTO."""

import argparse
import os
import re
import sys
import tempfile
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'urdf2webots'))
#  deepcode ignore C0413: We need to import the library first
from urdf2webots.importer import convert2urdf   # noqa: E402


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
    parser.add_argument('--multi-file', dest='enableMultiFile', action='store_true', default=False,
                        help='If set, the mesh files are exported as separated PROTO files')
    parser.add_argument('--static-base', dest='staticBase', action='store_true', default=False,
                        help='If set, the base link will have the option to be static (disable physics)')
    parser.add_argument('--tool-slot', dest='toolSlot', default=None,
                        help='Specify the link that you want to add a tool slot to (exact link name from urdf)')
    parser.add_argument('--rotation', dest='initRotation', default='0 1 0 0',
                        help='Set the rotation field of your PROTO file.)')
    parser.add_argument('--init-pos', dest='initPos', default=None,
                        help='Set the initial positions of your robot joints. '
                        'Example: --init-pos="[1.2, 0.5, -1.5]" would set '
                        'the first 3 joints of your robot to the specified values, '
                        'and leave the rest with their default value.')
    # use 'parse_known_args' because ROS2 adds a lot of internal arguments
    arguments, _ = parser.parse_known_args()
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
    urdf_file = file
    generated_file = False
    packages = re.findall(r'filename="package:\/\/([^\/]*)', content)
    for package in set(packages):  # do the replacement
        package_path = ''
        try:
            package_path = get_package_share_directory(package)
        except LookupError:
            sys.exit('This urdf depends on the "%s" package, but this package could not be found' %
                     package)
        content = content.replace('package://%s' % package, '%s' % package_path)
    if packages:  # some package-relative file paths have been found
        # generate a temporary file with the replacement content
        urdf_file = tempfile.mkstemp(suffix='.urdf')[1]
        generated_file = True
        with open(urdf_file, 'w') as f:
            f.write(content)
    convert2urdf(inFile=urdf_file,
                 outFile=arguments.outFile,
                 normal=arguments.normal,
                 boxCollision=arguments.boxCollision,
                 disableMeshOptimization=arguments.disableMeshOptimization,
                 enableMultiFile=arguments.enableMultiFile,
                 staticBase=arguments.staticBase,
                 toolSlot=arguments.toolSlot,
                 initRotation=arguments.initRotation,
                 initPos=arguments.initPos)
    # remove temporary file
    if generated_file:
        os.remove(urdf_file)


if __name__ == '__main__':
    main()
