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

"""Trajectory follower client for the UR5 robot used for multi-robot demonstration."""

import rclpy
from webots_ros2_universal_robot.follow_joint_trajectory_client import FollowJointTrajectoryClient


GOAL = {
    'joint_names': [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'finger_1_joint_1',
        'finger_2_joint_1',
        'finger_middle_joint_1'
    ],
    'points': [
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 0, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 3, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 4, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.85, 0.85, 0.6],
            'time_from_start': {'sec': 5, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 6, 'nanosec': 0}
        },
        {
            'positions': [0.63, -2.0, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 7, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 8, 'nanosec': 0}
        },
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 9, 'nanosec': 0}
        }
    ]
}


def main(args=None):
    rclpy.init(args=args)
    controller = FollowJointTrajectoryClient('ur5e_controller', '/ur5e/ur_joint_trajectory_controller')

    controller.send_goal(GOAL, 10)
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
