#!/usr/bin/env python

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

"""Base node class."""

import argparse
import os
import sys

from time import sleep

from webots_ros2_core.utils import get_webots_version, append_webots_python_lib_to_path

from rosgraph_msgs.msg import Clock

from rclpy.node import Node

try:
    append_webots_python_lib_to_path()
    from controller import Robot
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class WebotsNode(Node):

    def __init__(self, name, args=None):
        super().__init__(name)
        parser = argparse.ArgumentParser()
        parser.add_argument('--webots-robot-name', dest='webotsRobotName', default='',
                            help='Specifies the "name" field of the robot in Webots.')
        # use 'parse_known_args' because ROS2 adds a lot of internal arguments
        arguments, unknown = parser.parse_known_args()
        if arguments.webotsRobotName:
            os.environ['WEBOTS_ROBOT_NAME'] = arguments.webotsRobotName
        if get_webots_version() == 'R2019b':
            sleep(10)  # TODO: wait to make sure that Webots is started
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.clockPublisher = self.create_publisher(Clock, 'topic')
        timer_period = 0.001 * self.timestep  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.robot is None:
            return
        # Publish clock
        msg = Clock()
        time = self.robot.getTime()
        msg.clock.sec = int(time)
        # rounding prevents precision issues that can cause problems with ROS timers
        msg.clock.nanosec = int(round(1000 * (time - msg.clock.sec)) * 1.0e+6)
        self.clockPublisher.publish(msg)
        # Robot step
        if self.robot.step(self.timestep) < 0.0:
            del self.robot
            self.robot = None
            sys.exit(0)
