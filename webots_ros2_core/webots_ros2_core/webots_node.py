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

from webots_ros2_core.utils import append_webots_python_lib_to_path

from webots_ros2_msgs.srv import SetInt

from rosgraph_msgs.msg import Clock

from rclpy.node import Node

try:
    append_webots_python_lib_to_path()
    from controller import Supervisor
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class WebotsNode(Node):

    def __init__(self, name, args=None):
        super().__init__(name)
        self.declare_parameter('synchronization', False)
        parser = argparse.ArgumentParser()
        parser.add_argument('--webots-robot-name', dest='webotsRobotName', default='',
                            help='Specifies the "name" field of the robot in Webots.')
        # use 'parse_known_args' because ROS2 adds a lot of internal arguments
        arguments, unknown = parser.parse_known_args()
        if arguments.webotsRobotName:
            os.environ['WEBOTS_ROBOT_NAME'] = arguments.webotsRobotName
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.clockPublisher = self.create_publisher(Clock, 'clock', 10)
        timer_period = 0.001 * self.timestep  # seconds
        self.stepService = self.create_service(SetInt, 'step', self.step_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sec = 0
        self.nanosec = 0

    def step(self, ms):
        if self.robot is None or self.get_parameter('synchronization').value:
            return
        # Robot step
        if self.robot.step(ms) < 0.0:
            del self.robot
            self.robot = None
            sys.exit(0)
        # Update time
        time = self.robot.getTime()
        self.sec = int(time)
        # rounding prevents precision issues that can cause problems with ROS timers
        self.nanosec = int(round(1000 * (time - self.sec)) * 1.0e+6)
        # Publish clock
        msg = Clock()
        msg.clock.sec = self.sec
        msg.clock.nanosec = self.nanosec
        self.clockPublisher.publish(msg)

    def timer_callback(self):
        self.step(self.timestep)

    def step_callback(self, request, response):
        self.step(request.value)
        response.success = True
        return response
