#!/usr/bin/env python

# Copyright 1996-2022 Cyberbotics Ltd.
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

"""This process simply sends urdf information to the Spawner through a service."""

from launch.actions import ExecuteProcess


def get_webots_driver_node(event, driver_node):
    """Return the driver node in case the service response is successful."""
    if "success=True" in event.text.decode().strip():
        return driver_node
    print("WARNING: the Ros2Supervisor was not able to spawn this URDF robot.")
    return

class URDFSpawner(ExecuteProcess):
    def __init__(self, output='screen', name=None, urdf_path=None, translation='0 0 0', rotation='0 0 1 0', normal=False, box_collision=False, init_pos=None, **kwargs):
        command = [
                'ros2',
                'service',
                'call',
                '/spawn_urdf_robot',
                'webots_ros2_msgs/srv/SpawnUrdfRobot',
            ]

        message = '{ "robot": {'

        if name:
            message += '"name": "' + name + '",'
        if urdf_path:
            message += '"urdf_location": "' + urdf_path + '",'
        if translation:
            message += '"translation": "' + translation + '",'
        if rotation:
            message += '"rotation": "' + rotation + '",'
        if normal:
            message += '"normal": "' + str(normal) + '",'
        if box_collision:
            message += '"box_collision": "' + str(box_collision) + '",'
        if init_pos:
            message += '"init_pos": "' + init_pos + '",'

        message += '} }'
        command.append(message)

        super().__init__(
            output=output,
            cmd=command,
            **kwargs
        )
