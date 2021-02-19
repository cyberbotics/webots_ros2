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

"""Trajectory follower client for the ABB irb4600 robot."""

import rclpy
from webots_ros2_demos.follow_joint_trajectory_client import FollowJointTrajectoryClient


def main(args=None):
    rclpy.init(args=args)
    armed_robot_abb = FollowJointTrajectoryClient('armed_robots_abb', '/abb/follow_joint_trajectory')
    armed_robot_abb.send_goal({
        'joint_names': [
            'A motor',
            'B motor',
            'C motor',
            'E motor',
            'finger_1_joint_1',
            'finger_2_joint_1',
            'finger_middle_joint_1'
        ],
        'points': [
            {
                'positions': [0.0, 0.0, 0.0, 0., 0.0, 0.0, 0.0],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 0, 'nanosec': 0}
            },
            {
                'positions': [-0.025, 0.0, 0.82, -0.86, 0.0, 0.0, 0.0],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 1, 'nanosec': 0}
            },
            {
                'positions': [-0.025, 0.1, 0.82, -0.86, 0.0, 0.0, 0.0],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 2, 'nanosec': 0}
            },
            {
                'positions': [-0.025, 0.1, 0.82, -0.86, 0.85, 0.85, 0.6],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 3, 'nanosec': 0}
            },
            {
                'positions': [-0.025, -0.44, 0.82, -0.86, 0.85, 0.85, 0.6],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 4, 'nanosec': 0}
            },
            {
                'positions': [1.57, -0.1, 0.95, -0.71, 0.85, 0.85, 0.6],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 5, 'nanosec': 0}
            },
            {
                'positions': [1.57, -0.1, 0.8, -0.81, 0.0, 0.0, 0.0],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 6, 'nanosec': 0}
            },
            {
                'positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 7, 'nanosec': 0}
            },
            {
                'positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'velocities': [5] * 7,
                'accelerations': [5] * 7,
                'time_from_start': {'sec': 9, 'nanosec': 0}
            }
        ]
    }, 10)
    rclpy.spin(armed_robot_abb)


if __name__ == '__main__':
    main()
