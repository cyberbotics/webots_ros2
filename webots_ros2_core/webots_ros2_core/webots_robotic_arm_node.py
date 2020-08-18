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


import rclpy
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.joint_state_publisher import JointStatePublisher
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from webots_ros2_core.utils import get_node_name_from_args


class WebotsRoboticArmNode(WebotsNode):
    def __init__(self, name, args, prefix=''):
        super().__init__(name, args)
        self.prefix_param = self.declare_parameter('prefix', prefix)
        self.jointStatePublisher = JointStatePublisher(self.robot, self.prefix_param.value, self)
        self.trajectoryFollower = TrajectoryFollower(self.robot, self, jointPrefix=self.prefix_param.value)

        self.get_logger().info('Initializing robotic arm node with prefix = "%s"' % self.prefix_param.value)

    def step(self, ms):
        super().step(ms)
        # update joint state and trajectory follower
        self.jointStatePublisher.publish()


def main(args=None):
    rclpy.init(args=args)

    webots_robot_name = get_node_name_from_args()
    robotic_arm = WebotsRoboticArmNode(webots_robot_name, args=args)
    robotic_arm.start_device_manager()
    rclpy.spin(robotic_arm)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
