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

"""ROS2 TIAGo controller."""

import rclpy
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode


DEFAULT_WHEEL_DISTANCE = 0.404
DEFAULT_WHEEL_RADIUS = 0.1955


class TiagoController(WebotsDifferentialDriveNode):

    def __init__(self, args):
        super().__init__(
            'tiago_controller',
            args,
            wheel_distance=DEFAULT_WHEEL_DISTANCE,
            wheel_radius=DEFAULT_WHEEL_RADIUS,
            left_joint='wheel_left_joint',
            right_joint='wheel_right_joint',
            left_encoder='wheel_left_joint_sensor',
            right_encoder='wheel_right_joint_sensor',
        )


def main(args=None):
    rclpy.init(args=args)

    tiago_controller = TiagoController(args=args)

    rclpy.spin(tiago_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
