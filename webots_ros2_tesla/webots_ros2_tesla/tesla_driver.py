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

"""ROS2 Tesla driver."""

import rclpy
from ackermann_msgs.msg import AckermannDrive
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.webots.vehicle import Driver


class TeslaDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('tesla_driver', args, controller_class=Driver)
        self.start_device_manager()

        # ROS interface
        self.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)

    def __cmd_ackermann_callback(self, message):
        self.robot.setCruisingSpeed(message.speed)
        self.robot.setSteeringAngle(message.steering_angle)


def main(args=None):
    rclpy.init(args=args)
    driver = TeslaDriver(args=args)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
