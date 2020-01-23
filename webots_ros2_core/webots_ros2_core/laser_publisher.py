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

"""Laser publisher."""

import sys

from webots_ros2_core.utils import append_webots_python_lib_to_path

from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class LaserPublisher():
    """Publish as ROS topics the lidar laser scan."""

    def __init__(self, robot, node, prefix=''):
        """Initialize the position sensors and the topic."""
        self.robot = robot
        self.node = node
        self.lidars = []
        self.publishers = {}
        self.lastUpdate = {}
        self.timestep = int(robot.getBasicTimeStep())
        for i in range(robot.getNumberOfDevices()):
            device = robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.LIDAR:
                self.lidars.append(device)
                device.enable(self.timestep)  # TODO: variable time step
                self.publishers[device] = self.node.create_publisher(LaserScan, prefix + device.getName(), 1)  # TODO: variable names
                self.lastUpdate[device] = -100
        self.jointStateTimer = self.create_timer(0.001 * self.timestep, self.callback)

    def callback(self):
        for lidar in self.lidars:
            if self.robot.getTime() - self.lastUpdate[lidar] >= lidar.getSamplingPeriod():
                self.publish(lidar)

    def publish(self, lidar):
        """Publish the 'joint_states' topic with up to date value."""
        seconds = int(self.robot.getTime())
        nanoseconds = int((self.robot.getTime() - seconds) * 1.0e+6)
        msg = LaserScan()
        msg.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        msg.header.frame_id = 'From simulation state data'
        msg.angle_min = 0.0
        msg.angle_max = lidar.getFov()
        msg.angle_increment = lidar.getFov() / (lidar.getHorizontalResolution() - 1)
        msg.scan_time = lidar.getSamplingPeriod() / 1000.0
        msg.range_min = lidar.getMinRange()
        msg.range_max = lidar.getMaxRange()
        msg.ranges.resize(lidar.getHorizontalResolution())  # TODO: check if Python valid
        # msg.intensities.resize(lidar->getHorizontalResolution());

        lidarValues = lidar.getLayerRangeImage(0)  # TODO: multiple layers
        for i in range(lidar.getHorizontalResolution()):
            msg.ranges[i] = lidarValues[i]
        self.publishers[lidar].publish(msg)
