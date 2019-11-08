# Copyright 1996-2019 Cyberbotics Ltd.
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

"""ROS2 Universal Robots controller."""

from webots_ros2_demos.follow_joint_trajectory_client import followJointTrajectoryClient

import rclpy


def main(args=None):
    rclpy.init(args=args)
    armedRobotUR = followJointTrajectoryClient('/ur/follow_joint_trajectory')
    armedRobotUR.send_goal({
        'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                        'wrist_1_joint', 'finger_1_joint_1', 'finger_2_joint_1',
                        'finger_middle_joint_1'],
        'points': [
            {
                'positions': [0.0, 0.0, 0.0, 0.0, 0.85, 0.85, 0.6],
                'velocities': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'accelerations': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'time_from_start': {'sec': 4, 'nanosec': 0}
            },
            {
                'positions': [0.63, -2.26, -1.88, -2.14, 0.85, 0.85, 0.6],
                'velocities': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'accelerations': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'time_from_start': {'sec': 12, 'nanosec': 0}
            },
            {
                'positions': [0.63, -2.26, -1.88, -2.14, 0.0, 0.0, 0.0],
                'velocities': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'accelerations': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'time_from_start': {'sec': 16, 'nanosec': 0}
            },
            {
                'positions': [0.63, -2.0, -1.88, -2.14, 0.0, 0.0, 0.0],
                'velocities': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'accelerations': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'time_from_start': {'sec': 18, 'nanosec': 0}
            },
            {
                'positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'velocities': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'accelerations': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                'time_from_start': {'sec': 26, 'nanosec': 0}
            }
        ]
    })
    rclpy.spin(armedRobotUR)


if __name__ == '__main__':
    main()
