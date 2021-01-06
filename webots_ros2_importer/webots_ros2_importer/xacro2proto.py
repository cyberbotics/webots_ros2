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

"""Convert an XACRO File into a PROTO."""

import copy
import os
import sys
from tempfile import mkstemp
import xacro
from webots_ros2_importer import urdf2proto


def main(args=None):
    # remvove arguments specific to the urdf2proto converter
    saved_argv = copy.copy(sys.argv)
    args_to_remove = [
        '--normal',
        '--box-collision',
        '--disable-mesh-optimization',
        '--output',
        '--multi-file',
        '--static-base',
        '--tool-slot',
        '--rotation'
    ]
    for arg in saved_argv:
        for arg_to_remove in args_to_remove:
            if arg.startswith(arg_to_remove):
                sys.argv.remove(arg)
                break
    for arg in args_to_remove:
        if arg in sys.argv:
            sys.argv.remove(arg)
    # redirect stdout to temporary urdf file
    orig_stdout = sys.stdout
    _, path = mkstemp(suffix='.urdf')
    with open(path, 'w') as f:
        sys.stdout = f
        # run xacro to urdf conversio
        sys.argv.append('--inorder')
        xacro.main()
    # restore stdout and arguments and then run urdf to proto conversion
    sys.stdout = orig_stdout
    sys.argv = saved_argv
    urdf2proto.main(input=path)
    os.remove(path)


if __name__ == '__main__':
    main()
