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

from math import sin, cos
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from builtin_interfaces.msg import Time, Duration
import matplotlib.pyplot as plt
import numpy as np
from webots_ros2_core.math_utils import quaternion_to_euler


WORLD_WIDTH = 1
WORLD_HEIGHT = 1
RESOLUTION = 0.01

WORLD_ORIGIN_X = - WORLD_WIDTH / 2.0
WORLD_ORIGIN_Y = - WORLD_HEIGHT / 2.0
MAP_WIDTH = round(WORLD_WIDTH / RESOLUTION)
MAP_HEIGHT = round(WORLD_HEIGHT / RESOLUTION)


class SimpleMapper(Node):
    def __init__(self, name):
        super().__init__(name)

        self.map = [-1]*MAP_WIDTH*MAP_HEIGHT

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1, self.main_loop)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 1)
        self.scanner_subscriber = self.create_subscription(LaserScan, '/scan', self.update_map, 1)
        self.tf_publisher = TransformBroadcaster(self)

    def main_loop(self):
        now = self.get_clock().now()

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = RESOLUTION
        msg.info.width = MAP_WIDTH
        msg.info.height = MAP_HEIGHT
        msg.info.origin.position.x = WORLD_ORIGIN_X
        msg.info.origin.position.y = WORLD_ORIGIN_Y
        msg.data = self.map
        self.map_publisher.publish(msg)

    def update_map(self, msg):
        # Determine transformation of laser and robot in respect to odometry
        laser_rotation = None
        laser_translation = None
        try:
            tf = self.tf_buffer.lookup_transform(msg.header.frame_id, 'odom', Time(sec=0, nanosec=0))
            laser_rotation = quaternion_to_euler(tf.transform.rotation)[0]
            laser_translation = tf.transform.translation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print('No transformation found', e)
            return

        # Determine position of robot and laser
        world_robot_x = laser_translation.x + WORLD_ORIGIN_X
        world_robot_y = laser_translation.y + WORLD_ORIGIN_Y
        world_laser_xs = []
        world_laser_ys = []
        laser_range_angle = laser_rotation + msg.angle_min
        for laser_range in msg.ranges:
            if laser_range < msg.range_max and laser_range > msg.range_min:
                laser_x = world_robot_x + laser_range * cos(laser_range_angle)
                laser_y = world_robot_y + laser_range * sin(laser_range_angle)
                world_laser_xs.append(laser_x)
                world_laser_ys.append(laser_y)
            laser_range_angle += msg.angle_increment

        # Determine position on map (from world coordinates)
        robot_x = round(world_robot_x / RESOLUTION) - 1
        robot_y = round(world_robot_y / RESOLUTION) - 1
        laser_xs = []
        laser_ys = []
        for world_laser_x, world_laser_y in zip(world_laser_xs, world_laser_ys):
            laser_x = round(world_laser_x / RESOLUTION) - 1
            laser_y = round(world_laser_y / RESOLUTION) - 1
            laser_xs.append(laser_x)
            laser_ys.append(laser_y)

        # Bresenham's line algorithm (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
        for laser_x, laser_y in zip(laser_xs, laser_ys):
            delta_x = laser_x - robot_x
            delta_y = laser_y - robot_y
            delta_x += 1e-6 if delta_x == 0 else 0
            delta_err = abs(delta_y / delta_x)
            error = 0.0
            y = int(robot_y)
            for x in range(robot_x, laser_x, 1 if laser_x > robot_x else -1):
                self.map[x * MAP_WIDTH + y] = 0
                error = error + delta_err
                if error >= 0.5:
                    y += 1 if delta_y > 0 else -1
                    error -= 1.0
            self.map[laser_x * MAP_WIDTH + y] = 100


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = SimpleMapper('epuck_simple_mapper')
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
