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
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SimpleMapper(Node):
    def __init__(self, name):
        super().__init__(name)

        self.create_timer(1, self.main_loop)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 1)
        self.tf_publisher = TransformBroadcaster(self)

    def main_loop(self):
        now = self.get_clock().now()

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.01
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -(msg.info.width * msg.info.resolution / 2)
        msg.info.origin.position.y = -(msg.info.height * msg.info.resolution / 2)
        msg.data = [0]*(msg.info.width * msg.info.height)
        self.map_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = SimpleMapper('epuck_simple_mapper')
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
