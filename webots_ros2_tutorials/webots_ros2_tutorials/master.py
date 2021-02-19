# Copyright 1996-2021 Soft_illusion.
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
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class LineFollower(Node):
    def __init__(self):
        super().__init__('linefollower_cmdvel')
        # Subscribe Infra Red sensors
        self.subs_right_ir = self.create_subscription(
            Float64, 'right_IR', self.right_infrared_callback, 1)
        self.subs_left_ir = self.create_subscription(
            Float64, 'left_IR', self.left_infrared_callback, 1)
        self.subs_mid_ir = self.create_subscription(
            Float64, 'mid_IR', self.mid_infrared_callback, 1)
        # Publish cmd vel
        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)

        # vehicle parameters
        self.speed = 0.2
        self.angle_correction = 0.01

        # Initialize parameters
        self.ground_right, self.ground_mid, self.ground_left = 0, 0, 0
        self.delta = 0
        self.cmd = Twist()
        self.stop = False
        self.count = 0
        self.count_threshold = 10

    def lineFollowingModule(self):
        # Constant velocity
        self.cmd.linear.x = self.speed

        # Correction parameters
        self.delta = self.ground_right - self.ground_left
        self.cmd.angular.z = self.angle_correction*self.delta

        # Logic for stop if black line not seen .
        if self.ground_right > 500 and self.ground_left > 500 and self.ground_mid > 500:
            self.count += 1
        else:
            self.count = 0

        if self.count > self.count_threshold:
            self.stop = True

        if self.stop:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        # Publish cmd vel
        self.pubs_cmdvel.publish(self.cmd)
        self.stop = False

    # Call backs to update sensor reading variables
    def right_infrared_callback(self, msg):
        self.ground_right = msg.data
        self.lineFollowingModule()

    def left_infrared_callback(self, msg):
        self.ground_left = msg.data

    def mid_infrared_callback(self, msg):
        self.ground_mid = msg.data


def main(args=None):

    rclpy.init(args=args)

    ls = LineFollower()
    rclpy.spin(ls)

    ls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
