#!/usr/bin/env python

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

import sys
from webots_ros2_core.utils import append_webots_python_lib_to_path

try:
    append_webots_python_lib_to_path()
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e

#  deepcode ignore W0401,C0413: This module is just a proxy to `controller`
from controller import *    # noqa: F401,F403
