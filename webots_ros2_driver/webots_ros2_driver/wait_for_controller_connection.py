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

"""Start nodes when the controller connection is established with Webots."""


from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO


class WaitForControllerConnection(RegisterEventHandler):
    def __init__(self, target_driver, nodes_to_start):
        super().__init__(
            event_handler=OnProcessIO(
                target_action=target_driver,
                on_stderr=lambda event: self.on_stdout(event, nodes_to_start)
            )
        )

    def on_stdout(self, event, nodes_to_start):
        if 'Controller successfully connected to robot in Webots simulation.' in event.text.decode().strip():
            return nodes_to_start
        return
