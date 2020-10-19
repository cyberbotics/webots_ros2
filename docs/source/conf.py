#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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
import importlib


sys.path.insert(0, os.path.abspath('../../webots_ros2_core'))


autodoc_mock_imports = []
for mod in [
    'controller',
    'geometry_msgs',
    'launch',
    'launch.action',
    'launch.actions',
    'launch.launch_context',
    'launch.substitution',
    'launch.substitutions',
    'launch_ros',
    'nav_msgs',
    'rcl_interfaces',
    'rclpy',
    'rosgraph_msgs',
    'sensor_msgs',
    'std_msgs',
    'tf2_msgs',
    'tf2_ros',
    'tkinter',
    'webots_ros2_core',
    'webots_ros2_msgs'
]:
    try:
        importlib.import_module(mod)
    except ImportError:
        autodoc_mock_imports.append(mod)


project = 'webots_ros2'
copyright = '2020, Cyberbotics'
author = 'Cyberbotics'

extensions = [
    'sphinx_markdown_builder',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon'
]

master_doc = '_index'
