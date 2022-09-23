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
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'urdf2webots'))
#  deepcode ignore C0413: We need to import the library first
from urdf2webots.importer import convertUrdfFile   # noqa: E402


def main(args=None, urdfInput=None):
    parser = argparse.ArgumentParser(usage='usage: %prog --input=my_robot.urdf [options]')
    parser.add_option('--input', dest='input', default='', help='Specifies the URDF file.')
    parser.add_option('--output', dest='output', default='', help='Specifies the path and, if ending in ".proto", name '
                      'of the resulting PROTO file. The filename minus the .proto extension will be the robot name '
                      '(for PROTO conversion only).')
    parser.add_option('--normal', dest='normal', action='store_true', default=False,
                      help='If set, the normals are exported if present in the URDF definition.')
    parser.add_option('--box-collision', dest='boxCollision', action='store_true', default=False,
                      help='If set, the bounding objects are approximated using boxes.')
    parser.add_option('--tool-slot', dest='toolSlot', default=None,
                      help='Specify the link that you want to add a tool slot too (exact link name from URDF, for PROTO '
                      'conversion only).')
    parser.add_option('--translation', dest='initTranslation', default='0 0 0',
                      help='Set the translation field of your PROTO file or Webots VRML robot string.')
    parser.add_option('--rotation', dest='initRotation', default='0 0 1 0',
                      help='Set the rotation field of your PROTO file or Webots Robot node string.')
    parser.add_option('--init-pos', dest='initPos', default=None,
                      help='Set the initial positions of your robot joints. Example: --init-pos="[1.2, 0.5, -1.5]" would '
                      'set the first 3 joints of your robot to the specified values, and leave the rest with their '
                      'default value.')
    parser.add_option('--link-to-def', dest='linkToDef', action='store_true', default=False,
                      help='Creates a DEF with the link name for each solid to be able to access it using '
                      'getFromProtoDef(defName) (for PROTO conversion only).')
    parser.add_option('--joint-to-def', dest='jointToDef', action='store_true', default=False,
                      help='Creates a DEF with the joint name for each joint to be able to access it using '
                      'getFromProtoDef(defName) (for PROTO conversion only).')
    # use 'parse_known_args' because ROS2 adds a lot of internal arguments
    arguments, _ = parser.parse_known_args()
    file = os.path.abspath(urdfInput) if urdfInput is not None else os.path.abspath(arguments.inputConversion)
    if not file:
        sys.exit('Input file not specified (should be specified with the "--input" argument).')
    if not os.path.isfile(file):
        sys.exit('"%s" file does not exist.' % arguments.inputConversion)
    elif not file.endswith('.urdf'):
        sys.exit('"%s" is not an urdf file.' % file)
    convertUrdfFile(input=arguments.inputConversion,
                    output=arguments.output,
                    normal=arguments.normal,
                    boxCollision=arguments.boxCollision,
                    toolSlot=arguments.toolSlot,
                    initTranslation=arguments.initTranslation,
                    initRotation=arguments.initRotation,
                    initPos=arguments.initPos,
                    linkToDef=arguments.linkToDef,
                    jointToDef=arguments.jointToDef)


if __name__ == '__main__':
    main()
