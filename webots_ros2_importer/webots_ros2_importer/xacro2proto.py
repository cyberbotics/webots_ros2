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

"""Convert an XACRO File into a PROTO."""

import argparse
import copy
import os
import sys
from tempfile import mkstemp
import xacro
from webots_ros2_importer import urdf2proto


def main(args=None):
    parser = argparse.ArgumentParser(usage='usage: %(prog)s --input=my_robot.urdf.xacro [options]')
    parser.add_argument('--input', dest='inFile', default=None,
                        help='Specifies the xacro file to convert.')
    parser.add_argument('--xacro-opts', dest='xacro_opts', default=None,
                        help='If set, these options are forwarded to the xacro to urdf converter')

    # use 'parse_known_args' because ROS2 adds a lot of internal arguments
    arguments, _ = parser.parse_known_args()

    saved_argv = copy.copy(sys.argv)

    if arguments.inFile:
        file = os.path.abspath(arguments.inFile)
        if not os.path.isfile(file):
            sys.exit('"%s" file does not exist.' % arguments.inFile)
        elif not file.endswith('.xacro'):
            sys.exit('"%s" is not a xacro file.' % file)

        xacro_opts = arguments.xacro_opts.split(" ") if arguments.xacro_opts is not None else []

        sys.argv = [sys.argv[0]] + xacro_opts + [file]
    else:
        print(
            'Input file not specified with the "--input" argument, '
            'using the last arguments as input to the xacro to urdf converter.'
        )

        # remove arguments specific to the urdf2proto converter
        args_for_urdf2proto = [
            '--output',
            '--normal',
            '--box-collision',
            '--disable-mesh-optimization',
            '--multi-file',
            '--tool-slot',
            '--translation',
            '--rotation',
            '--init-pos',
            '--link-to-def',
            '--joint-to-def'
        ]
        for arg in saved_argv:
            for arg_for_urdf2proto in args_for_urdf2proto:
                if arg.startswith(arg_for_urdf2proto):
                    sys.argv.remove(arg)
                    break

    # redirect stdout to temporary urdf file
    orig_stdout = sys.stdout
    _, path = mkstemp(suffix='.urdf')
    with open(path, 'w') as f:
        sys.stdout = f
        # run xacro to urdf conversion
        xacro.main()

    # restore stdout and arguments and then run urdf to proto conversion
    sys.stdout = orig_stdout
    sys.argv = saved_argv
    urdf2proto.main(urdfInput=path)
    os.remove(path)


if __name__ == '__main__':
    main()
