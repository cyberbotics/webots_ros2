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

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped


import transforms3d

try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class LaserPublisher():
    """Publish as ROS topics the laser scans of the lidars."""

    def __init__(self, robot, node, prefix='', parameters={}):
        """
        Initialize the lidars and the topics.

        Arguments:
        prefix: prefix for the topic names
        parameters: customization parameters dictionnary the key are the device names
                    the value are dictionaries with the following key:
                        'timestep' and 'topic name'
        """
        self.robot = robot
        self.node = node
        self.prefix = prefix
        self.lidars = []
        self.publishers = {}
        self.lastUpdate = {}
        self.timestep = int(robot.getBasicTimeStep())
        self.tfPublisher = self.node.create_publisher(TFMessage, 'tf', 10)
        for i in range(robot.getNumberOfDevices()):
            device = robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.LIDAR:
                self.lidars.append(device)
                if device.getName() in parameters and 'timestep' in parameters[device.getName()]:
                    device.enable(parameters[device.getName()]['timestep'])
                else:
                    device.enable(self.timestep)
                topicName = device.getName()
                if device.getName() in parameters and 'topic name' in parameters[topicName]:
                    topicName = parameters[topicName]['topic name']
                if device.getNumberOfLayers() > 1:
                    self.publishers[device] = {}
                    for j in range(device.getNumberOfLayers()):
                        name = prefix + device.getName() + '_' + str(j)
                        indexedTopicName = prefix + topicName + '_' + str(j)
                        publisher = self.node.create_publisher(LaserScan, indexedTopicName, 1)
                        self.publishers[device][name] = publisher
                else:
                    self.publishers[device] = self.node.create_publisher(LaserScan,
                                                                         prefix + topicName, 1)
                self.lastUpdate[device] = -100
        self.jointStateTimer = self.node.create_timer(0.001 * self.timestep, self.callback)

    def callback(self):
        tFMessage = TFMessage()
        for lidar in self.lidars:
            if self.robot.getTime() - self.lastUpdate[lidar] >= lidar.getSamplingPeriod():
                self.publish(lidar, tFMessage.transforms)
        if tFMessage.transforms:
            self.tfPublisher.publish(tFMessage)

    def publish(self, lidar, transforms):
        """Publish the laser scan topics with up to date value."""
        stamp = Time(seconds=self.robot.getTime() + 0.001 * self.timestep).to_msg()
        for i in range(lidar.getNumberOfLayers()):
            name = self.prefix + lidar.getName() + '_scan'
            if lidar.getNumberOfLayers() > 1:
                name += '_' + str(i)
            # publish the lidar to scan transform
            transformStamped = TransformStamped()
            transformStamped.header.stamp = stamp
            transformStamped.header.frame_id = self.prefix + lidar.getName()
            transformStamped.child_frame_id = name
            q1 = transforms3d.quaternions.axangle2quat([0, 1, 0], -1.5708)
            q2 = transforms3d.quaternions.axangle2quat([1, 0, 0], 1.5708)
            result = transforms3d.quaternions.qmult(q1, q2)
            if lidar.getNumberOfLayers() > 1:
                angleStep = lidar.getVerticalFov() / (lidar.getNumberOfLayers() - 1)
                angle = -0.5 * lidar.getVerticalFov() + i * angleStep
                q3 = transforms3d.quaternions.axangle2quat([0, 0, 1], angle)
                result = transforms3d.quaternions.qmult(result, q3)
            transformStamped.transform.rotation.x = result[0]
            transformStamped.transform.rotation.y = result[1]
            transformStamped.transform.rotation.z = result[2]
            transformStamped.transform.rotation.w = result[3]
            transforms.append(transformStamped)
            # publish the actual laser scan
            msg = LaserScan()
            msg.header.stamp = stamp
            msg.header.frame_id = name
            msg.angle_min = -0.5 * lidar.getFov()
            msg.angle_max = 0.5 * lidar.getFov()
            msg.angle_increment = lidar.getFov() / (lidar.getHorizontalResolution() - 1)
            msg.scan_time = lidar.getSamplingPeriod() / 1000.0
            msg.range_min = lidar.getMinRange()
            msg.range_max = lidar.getMaxRange()
            lidarValues = lidar.getLayerRangeImage(i)
            for j in range(lidar.getHorizontalResolution()):
                msg.ranges.append(lidarValues[j])
            if lidar.getNumberOfLayers() > 1:
                key = self.prefix + lidar.getName() + '_' + str(i)
                self.publishers[lidar][key].publish(msg)
            else:
                self.publishers[lidar].publish(msg)
