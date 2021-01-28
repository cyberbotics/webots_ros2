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

"""ROS2 Mavic 2 Pro driver."""

import rclpy
from webots_ros2_core.webots_node import WebotsNode


class MavicDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('mavic_driver', args)
        self.start_device_manager()

        # Sensors
        self.__gps = self.robot.getDevice('gps')
        self.__gyro = self.robot.getDevice('gyro')
        self.__compass = self.robot.getDevice('compass')

        # Propellers
        self.__propellers = [
            self.robot.getDevice('front left propeller'),
            self.robot.getDevice('front right propeller'),
            self.robot.getDevice('rear left propeller'),
            self.robot.getDevice('rear right propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(1)

        # State
        self.__target_twist = None

    def step(self, ms):
        super().step(ms)

        if self.__target_twist is None:
            return

        # Control


def main(args=None):
    rclpy.init(args=args)
    driver = MavicDriver(args=args)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
