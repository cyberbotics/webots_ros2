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

"""ROS2 ABB robots controller."""

from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.joint_state_publisher import JointStatePublisher
from webots_ros2_core.trajectory_follower import TrajectoryFollower

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


class ActionServerNode(WebotsNode):

    def __init__(self, args):
        super().__init__('abb_driver', args=args)
        prefix = self.get_parameter_or('prefix',
                                       Parameter('prefix', Parameter.Type.STRING, '')).value
        self.jointStatePublisher = JointStatePublisher(self.robot, prefix, self)
        self.trajectoryFollower = TrajectoryFollower(self.robot, self, jointPrefix=prefix)
        self.jointStateTimer = self.create_timer(0.001 * self.timestep, self.joint_state_callback)

    def joint_state_callback(self):
        # update joint state and trajectory follower
        self.jointStatePublisher.publish()


def main(args=None):
    rclpy.init(args=args)

    actionServer = ActionServerNode(args=args)

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(actionServer, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actionServer.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
