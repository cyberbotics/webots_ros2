#!/usr/bin/env python

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

"""This process simply sends urdf information to the Spawner through a service."""

from launch.actions import ExecuteProcess


def get_webots_driver_node(event, driver_node):
    """Return the driver node in case the service response is successful."""
    if 'success=True' in event.text.decode().strip():
        return driver_node
    if 'success=False' in event.text.decode().strip():
        print('WARNING: the Ros2Supervisor was not able to spawn this URDF robot.')
    return


class PROTOSpawner(ExecuteProcess):
    def __init__(self, output='log', name=None, proto_path=None, robot_string=None, **kwargs):
        message = '{robot: {'

        if proto_path:
            message += 'proto_path: "' + proto_path + '",'
        elif robot_string:
            message += 'robot_string: "\\\n' + robot_string + '",'

        message += '} }'

        command = ['ros2',
                   'service',
                   'call',
                   '/Ros2Supervisor/spawn_urdf_robot',
                   'webots_ros2_msgs/srv/SpawnProtoRobot',
                   message]

        super().__init__(
            output=output,
            cmd=command,
            **kwargs
        )
