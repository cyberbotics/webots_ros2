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


class Device:
    def step(self):
        raise NotImplementedError('.step() method is called on every step and it should be implemented.')

    def _create_topic_name(self, wb_device):
        return wb_device.getName().replace('-', '_').replace(' ', '_')

    def _create_frame_id(self, wb_device):
        return self._create_topic_name(wb_device)
