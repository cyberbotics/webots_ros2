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

"""This launcher simply starts Webots."""

import os
import sys
from launch.actions import ExecuteProcess
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from webots_ros2_driver.utils import get_webots_home, handle_webots_installation


import shutil
import pathlib

from typing import List
from typing import Optional
from typing import Iterable

from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node



from urdf2webots.importer import convert2urdf

'''
def webotsCustomLaunchDescription(initial_entities: List[LaunchDescriptionEntity]):
    launch_description = LaunchDescription(initial_entities)

    # find WebotsLauncher
    for entity in initial_entities:
        if type(entity) == WebotsLauncher:
            for robot in WebotsLauncher(entity).__robots:
                robot_driver = Node(
                    package='webots_ros2_driver',
                    executable='driver',
                    output='screen',
                    additional_env={'WEBOTS_ROBOT_NAME': robot.get('name')},
                    parameters=[
                        {'robot_description': pathlib.Path(robot.get('urdf_location')).read_text()},
                    ]
                )

                launch_description.add_entity(robot_driver)

    return launch_description
'''

class _ConditionalSubstitution(Substitution):
    def __init__(self, *, condition, false_value='', true_value=''):
        self.__condition = condition if isinstance(condition, Substitution) else TextSubstitution(text=str(condition))
        self.__false_value = false_value if isinstance(false_value, Substitution) else TextSubstitution(text=false_value)
        self.__true_value = true_value if isinstance(true_value, Substitution) else TextSubstitution(text=true_value)

    def perform(self, context):
        if context.perform_substitution(self.__condition).lower() in ['false', '0', '']:
            return context.perform_substitution(self.__false_value)
        return context.perform_substitution(self.__true_value)


class WebotsLauncher(ExecuteProcess):
    def __init__(self, output='screen', world=None, robots=[], gui=True, mode='realtime', stream=False, **kwargs):
        # Find Webots executable
        webots_path = get_webots_home(show_warning=True)
        if webots_path is None:
            handle_webots_installation()
            webots_path = get_webots_home()
        if sys.platform == 'win32':
            webots_path = os.path.join(webots_path, 'msys64', 'mingw64', 'bin')
        webots_path = os.path.join(webots_path, 'webots')

        mode = mode if isinstance(mode, Substitution) else TextSubstitution(text=mode)
        world = world if isinstance(world, Substitution) else TextSubstitution(text=world)

        self.__world = world
        self.__robots = robots

        no_rendering = _ConditionalSubstitution(condition=gui, false_value='--no-rendering')
        stdout = _ConditionalSubstitution(condition=gui, false_value='--stdout')
        stderr = _ConditionalSubstitution(condition=gui, false_value='--stderr')
        no_sandbox = _ConditionalSubstitution(condition=gui, false_value='--no-sandbox')
        if sys.platform == 'win32':
            # Windows doesn't have the sandbox argument
            no_sandbox = ''
        minimize = _ConditionalSubstitution(condition=gui, false_value='--minimize')
        stream_argument = _ConditionalSubstitution(condition=stream, true_value='--stream')

        xvfb_run_prefix = []
        if 'WEBOTS_OFFSCREEN' in os.environ:
            xvfb_run_prefix.append('xvfb-run')
            xvfb_run_prefix.append('--auto-servernum')

        # no_rendering, stdout, stderr, no_sandbox, minimize
        super().__init__(
            output=output,
            cmd=xvfb_run_prefix + [
                webots_path,
                stream_argument,
                no_rendering,
                stdout,
                stderr,
                no_sandbox,
                minimize,
                world,
                '--batch',
                ['--mode=', mode],
            ],
            **kwargs
        )

    def execute(self, context: LaunchContext):
        # Check if the user wants to convert URDF files into robots
        if self.__robots:
            world_path = self.__world.perform(context)
            if not world_path:
                sys.exit('World file not specified (has to be specified with world=path/to/my/world.wbt')

            world_copy = world_path[:-4] + '_with_URDF_robot.wbt'
            shutil.copyfile(world_path, world_copy)
            world_name=context.launch_configurations['world']
            context.launch_configurations['world']=world_name[:-4] + '_with_URDF_robot.wbt'

            for robot in self.__robots:
                file_input = robot.get('urdf_location')
                robot_name = robot.get('name')
                robot_translation = robot.get('translation')
                robot_rotation = robot.get('rotation')

                if not file_input:
                    sys.exit('URDF file not specified (has to be specified with \'urdf_location\': \'path/to/my/robotUrdf.urdf\'')
                if not robot_name:
                    sys.stderr.write('Robot name not specified (should be specified if more than one robot is present with \'name\': \'robotName\'\n')

                convert2urdf(inFile=file_input, worldFile=world_copy, robotName=robot_name, initTranslation=robot_translation, initRotation=robot_rotation)

        return super().execute(context)

    def getDriversList(self):
        nodes_list = []

        for robot in self.__robots:

            parameters=[
                {'robot_description': pathlib.Path(robot.get('urdf_location')).read_text()},
                {'use_sim_time': robot.get('use_sim_time')},
                
            ]

            if robot.get('control_params'):
                parameters.append(robot.get('control_params'))
            
            robot_driver = Node(
                package='webots_ros2_driver',
                executable='driver',
                output='screen',
                additional_env={'WEBOTS_ROBOT_NAME': robot.get('name')},
                parameters=parameters,
                remappings=robot.get('remappings')
            )

            nodes_list.append(robot_driver)

        return nodes_list
