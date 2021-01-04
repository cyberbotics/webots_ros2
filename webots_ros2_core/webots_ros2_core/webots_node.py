#!/usr/bin/env python

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

"""Base node class."""

import os
import sys
import argparse

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from webots_ros2_msgs.srv import SetInt
from webots_ros2_core.joint_state_publisher import JointStatePublisher
from webots_ros2_core.devices.device_manager import DeviceManager
from webots_ros2_core.utils import get_node_name_from_args

from webots_ros2_core.webots_controller import Supervisor


class WebotsNode(Node):
    """
    Extends ROS2 base node to provide integration with Webots.

    Args:
        name (WebotsNode): Webots Robot node.
        args (dict): Arguments passed to ROS2 base node.
    """

    def __init__(self, name, args=None):
        super().__init__(name)
        self.declare_parameter('synchronization', False)
        self.declare_parameter('use_joint_state_publisher', False)
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '--webots-robot-name',
            dest='webots_robot_name',
            default='',
            help='Specifies the "name" field of the robot in Webots.'
        )

        # Get robot name
        arguments, _ = parser.parse_known_args()
        if arguments.webots_robot_name:
            os.environ['WEBOTS_ROBOT_NAME'] = arguments.webots_robot_name

        self.robot = Supervisor()
        self.__timestep = int(self.robot.getBasicTimeStep())
        self.__clock_publisher = self.create_publisher(Clock, 'clock', 10)
        self.__step_service = self.create_service(SetInt, 'step', self.__step_callback)
        self.__timer = self.create_timer(0.001 * self.__timestep, self.__timer_callback)
        self.__device_manager = None

        # Joint state publisher
        self.__joint_state_publisher = None
        if self.get_parameter('use_joint_state_publisher').value:
            self.__joint_state_publisher = JointStatePublisher(self.robot, '', self)

    def start_joint_state_publisher(self):
        """
        Use `JointStatePublisher` to publish ROS2 messages of type
            [`sensor_msgs/JointState`](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg).
        """
        self.__joint_state_publisher = JointStatePublisher(self.robot, '', self)

    def step(self, ms):
        if self.get_parameter('use_joint_state_publisher').value:
            self.__joint_state_publisher.publish()
        if self.__device_manager:
            self.__device_manager.step()
        if self.robot is None or self.get_parameter('synchronization').value:
            return

        # Robot step
        if self.robot.step(ms) < 0.0:
            del self.robot
            self.robot = None
            sys.exit(0)

        # Update time
        msg = Clock()
        msg.clock = Time(seconds=self.robot.getTime()).to_msg()
        self.__clock_publisher.publish(msg)

    def __timer_callback(self):
        self.step(self.__timestep)

    def __step_callback(self, request, response):
        self.step(request.value)
        response.success = True
        return response

    def start_device_manager(self, config=None):
        """
        Start automatic ROSification of Webots devices available in the robot.

        Kwargs:
            config (dict): Dictionary of properties in format::

                {
                    [device_name]: {
                        [property_name]: [property_value]
                    }
                }

        """
        self.__device_manager = DeviceManager(self, config)


def main(args=None):
    rclpy.init(args=args)

    webots_robot_name = get_node_name_from_args()
    driver = WebotsNode(webots_robot_name, args=args)
    driver.start_device_manager()
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
