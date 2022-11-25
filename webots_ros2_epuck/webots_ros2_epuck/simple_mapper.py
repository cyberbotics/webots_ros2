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

from math import sin, cos, atan2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from builtin_interfaces.msg import Time


WORLD_WIDTH = 3
WORLD_HEIGHT = 3
RESOLUTION = 0.01

WORLD_ORIGIN_X = - WORLD_WIDTH / 2.0
WORLD_ORIGIN_Y = - WORLD_HEIGHT / 2.0
MAP_WIDTH = int(WORLD_WIDTH / RESOLUTION)
MAP_HEIGHT = int(WORLD_HEIGHT / RESOLUTION)


class SimpleMapper(Node):
    def __init__(self, name):
        super().__init__(name)

        fill_map_param = self.declare_parameter('fill_map', True)

        # Init map related elements
        self.map = [-1] * MAP_WIDTH * MAP_HEIGHT
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        )
        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        # Init laser related elements
        if fill_map_param.value:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.scanner_subscriber = self.create_subscription(LaserScan, '/scan', self.update_map, 1)

        # Start publishing the map
        self.publish_map()
        self.create_timer(1, self.publish_map)

    def publish_map(self):
        now = self.get_clock().now()

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
            tf = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, Time(sec=0, nanosec=0))
            q = tf.transform.rotation
            laser_rotation = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            laser_translation = tf.transform.translation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print('No required transformation found: `{}`'.format(str(e)))
            return

        # Determine position of robot and laser
        world_robot_x = laser_translation.x + WORLD_ORIGIN_X
        world_robot_y = laser_translation.y + WORLD_ORIGIN_Y
        world_laser_xs = []
        world_laser_ys = []
        laser_range_angle = msg.angle_min + laser_rotation
        for laser_range in msg.ranges:
            if laser_range < msg.range_max and laser_range > msg.range_min:
                laser_x = world_robot_x + laser_range * cos(laser_range_angle)
                laser_y = world_robot_y + laser_range * sin(laser_range_angle)
                world_laser_xs.append(laser_x)
                world_laser_ys.append(laser_y)
            laser_range_angle += msg.angle_increment

        # Determine position on map (from world coordinates)
        robot_x = int(world_robot_x / RESOLUTION)
        robot_y = int(world_robot_y / RESOLUTION)
        laser_xs = []
        laser_ys = []
        for world_laser_x, world_laser_y in zip(world_laser_xs, world_laser_ys):
            laser_x = int(world_laser_x / RESOLUTION)
            laser_y = int(world_laser_y / RESOLUTION)
            laser_xs.append(laser_x)
            laser_ys.append(laser_y)

        # Fill the map based on known readings
        for laser_x, laser_y in zip(laser_xs, laser_ys):
            self.plot_bresenham_line(robot_x, laser_x, robot_y, laser_y)
            self.map[laser_y * MAP_WIDTH + laser_x] = 100

    def plot_bresenham_line(self, x0, x1, y0, y1):
        # Bresenham's line algorithm (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.map[y0 * MAP_WIDTH + x0] = 0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = SimpleMapper('epuck_simple_mapper')
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
