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

"""ROS2 e-puck driver."""

from math import pi
import rclpy
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

OUT_OF_RANGE = 0.0
INFRARED_MAX_RANGE = 0.04
INFRARED_MIN_RANGE = 0.009
TOF_MAX_RANGE = 1.0
NB_INFRARED_SENSORS = 8
SENSOR_DIST_FROM_CENTER = 0.035


DISTANCE_SENSOR_ANGLE = [
    -15 * pi / 180,   # ps0
    -45 * pi / 180,   # ps1
    -90 * pi / 180,   # ps2
    -150 * pi / 180,  # ps3
    150 * pi / 180,   # ps4
    90 * pi / 180,    # ps5
    45 * pi / 180,    # ps6
    15 * pi / 180,    # ps7
]


def interpolate_function(value, start_x, start_y, end_x, end_y, ascending=None):
    if end_x - start_x == 0:
        if (ascending and value < start_x) or (not ascending and value > start_x):
            return start_y
        elif (ascending and value > start_x) or (not ascending and value < start_x):
            return end_y
        else:
            return start_y + (end_y - start_y) / 2
    slope = (end_y - start_y) / (end_x - start_x)
    return slope * (value - start_x) + start_y


def interpolate_lookup_table(value, table):
    if not table:
        return value

    # Interpolate
    for i in range(int(len(table) / 3) - 1):
        if (value < table[i * 3 + 1] and value >= table[(i + 1) * 3 + 1]) or \
                (value >= table[i * 3 + 1] and value < table[(i + 1) * 3 + 1]):
            return interpolate_function(
                value,
                table[i * 3 + 1],
                table[i * 3],
                table[(i + 1) * 3 + 1],
                table[(i + 1) * 3]
            )

    # Extrapolate (we assume that the table is sorted, order is irrelevant)
    ascending = (table[1] < table[len(table) - 1*3 + 1])
    if (ascending and value >= table[1]) or (not ascending and value < table[1]):
        return interpolate_function(
            value,
            table[len(table) - 2 * 3 + 1],
            table[len(table) - 2 * 3 + 0],
            table[len(table) - 1 * 3 + 1],
            table[len(table) - 1 * 3 + 0],
            ascending
        )
    else:
        return interpolate_function(
            value,
            table[1],
            table[0],
            table[3 + 1],
            table[3],
            ascending
        )

class EPuckNode(Node):
    def __init__(self):
        super().__init__('epuck_node')
        self.get_logger().info("Epuck node has been started.")

        # Intialize distance sensors for LaserScan topic
        self.__subscriber_distance_sensors = {}
        self.__dists = {}
        for i in range(NB_INFRARED_SENSORS):
            self.__dists['ps{}'.format(i)] = OUT_OF_RANGE

        self.__subscriber_distance_sensors['ps0'] = self.create_subscription(Range,'/e_puck/ps0', self.__process_dist_sens_0, 1)
        self.__subscriber_distance_sensors['ps1'] = self.create_subscription(Range,'/e_puck/ps1', self.__process_dist_sens_1, 1)
        self.__subscriber_distance_sensors['ps2'] = self.create_subscription(Range,'/e_puck/ps2', self.__process_dist_sens_2, 1)
        self.__subscriber_distance_sensors['ps3'] = self.create_subscription(Range,'/e_puck/ps3', self.__process_dist_sens_3, 1)
        self.__subscriber_distance_sensors['ps4'] = self.create_subscription(Range,'/e_puck/ps4', self.__process_dist_sens_4, 1)
        self.__subscriber_distance_sensors['ps5'] = self.create_subscription(Range,'/e_puck/ps5', self.__process_dist_sens_5, 1)
        self.__subscriber_distance_sensors['ps6'] = self.create_subscription(Range,'/e_puck/ps6', self.__process_dist_sens_6, 1)
        self.__subscriber_distance_sensors['ps7'] = self.create_subscription(Range,'/e_puck/ps7', self.__process_dist_sens_7, 1)

        self.__subscriber_tof = self.create_subscription(Range, '/e_puck/tof', self.__process_tof, 1)
        self.__new_tof_data = False

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.get_clock().now().to_msg()
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser_scanner'
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.033

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(laser_transform)

        # Main loop self.get_clock
        self.create_timer(100 / 1000, self.__publish_laserscan_data)

    def __process_dist_sens_0(self, msg):
        self.__dists["ps0"] = msg.range

    def __process_dist_sens_1(self, msg):
        self.__dists["ps1"] = msg.range

    def __process_dist_sens_2(self, msg):
        self.__dists["ps2"] = msg.range

    def __process_dist_sens_3(self, msg):
        self.__dists["ps3"] = msg.range

    def __process_dist_sens_4(self, msg):
        self.__dists["ps4"] = msg.range

    def __process_dist_sens_5(self, msg):
        self.__dists["ps5"] = msg.range

    def __process_dist_sens_6(self, msg):
        self.__dists["ps6"] = msg.range

    def __process_dist_sens_7(self, msg):
        self.__dists["ps7"] = msg.range

    def __process_tof(self, msg):
        self.__new_tof_data = True
        self.__tof_value = msg.range

    def __publish_laserscan_data(self):
        stamp = self.get_clock().now().to_msg()
        dists = [OUT_OF_RANGE] * NB_INFRARED_SENSORS
        dist_tof = OUT_OF_RANGE

        # Calculate distances
        for i, key in enumerate(self.__dists):
            dists[i] = self.__dists[key]

        # Publish range: ToF
        if self.__new_tof_data:
            dist_tof = self.__tof_value
            self.__new_tof_data = False
        else:
            dist_tof = OUT_OF_RANGE

        # Max range of ToF sensor is 2m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        laser_dists = [OUT_OF_RANGE if dist > INFRARED_MAX_RANGE else dist for dist in dists]
        msg = LaserScan()
        msg.header.frame_id = 'laser_scanner'
        msg.header.stamp = stamp
        msg.angle_min = - 150 * pi / 180
        msg.angle_max = 150 * pi / 180
        msg.angle_increment = 15 * pi / 180
        msg.range_min = SENSOR_DIST_FROM_CENTER + INFRARED_MIN_RANGE
        msg.range_max = SENSOR_DIST_FROM_CENTER + TOF_MAX_RANGE
        msg.ranges = [
            laser_dists[3] + SENSOR_DIST_FROM_CENTER,   # -150
            OUT_OF_RANGE,                               # -135
            OUT_OF_RANGE,                               # -120
            OUT_OF_RANGE,                               # -105
            laser_dists[2] + SENSOR_DIST_FROM_CENTER,   # -90
            OUT_OF_RANGE,                               # -75
            OUT_OF_RANGE,                               # -60
            laser_dists[1] + SENSOR_DIST_FROM_CENTER,   # -45
            OUT_OF_RANGE,                               # -30
            laser_dists[0] + SENSOR_DIST_FROM_CENTER,   # -15
            dist_tof + SENSOR_DIST_FROM_CENTER,         # 0
            laser_dists[7] + SENSOR_DIST_FROM_CENTER,   # 15
            OUT_OF_RANGE,                               # 30
            laser_dists[6] + SENSOR_DIST_FROM_CENTER,   # 45
            OUT_OF_RANGE,                               # 60
            OUT_OF_RANGE,                               # 75
            laser_dists[5] + SENSOR_DIST_FROM_CENTER,   # 90
            OUT_OF_RANGE,                               # 105
            OUT_OF_RANGE,                               # 120
            OUT_OF_RANGE,                               # 135
            laser_dists[4] + SENSOR_DIST_FROM_CENTER,   # 150
        ]
        self.laser_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckNode()
    rclpy.spin(epuck_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    epuck_controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
