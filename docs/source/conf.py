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

import os
import sys
import datetime
import importlib
from unittest.mock import MagicMock


sys.path.insert(0, os.path.abspath('../../webots_ros2_core'))


# Mock imports
autodoc_mock_imports = []
for mod in [
    'controller',
    'control_msgs',
    'geometry_msgs',
    'launch',
    'launch.action',
    'launch.actions',
    'launch.launch_context',
    'launch.substitution',
    'launch.substitutions',
    'launch_ros',
    'nav_msgs',
    'rclpy',
    'rcl_interfaces',
    'rosgraph_msgs',
    'sensor_msgs',
    'std_msgs',
    'tf2_msgs',
    'tf2_ros',
    'trajectory_msgs',
    'tkinter',
    'webots_ros2_core',
    'webots_ros2_msgs'
]:
    try:
        importlib.import_module(mod)
    except ImportError:
        autodoc_mock_imports.append(mod)


# Mock inherited classes
MOCK_MODULES = [
    'rclpy.node',
]

MOCK_CLASSES = [
    'Node',
]


class Mock(MagicMock):
    @classmethod
    def __getattr__(cls, name):
        if name in MOCK_CLASSES:
            return object
        return MagicMock()


sys.modules.update((mod_name, Mock()) for mod_name in MOCK_MODULES)


# Meta data
project = 'webots_ros2'
#  deepcode ignore W0622: Ignore W0622
copyright = '{}, Cyberbotics'.format(datetime.datetime.now().year)
author = 'Cyberbotics'

# Extensions
extensions = [
    'sphinx_markdown_builder',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon'
]

# Misc
master_doc = '_API-Index'
