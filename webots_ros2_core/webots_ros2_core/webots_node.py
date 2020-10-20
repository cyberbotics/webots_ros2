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
from rclpy.time import Time
from webots_ros2_core.joint_state_publisher import JointStatePublisher
from webots_ros2_core.devices.device_manager import DeviceManager

from webots_ros2_core.utils import append_webots_python_lib_to_path, get_node_name_from_args
from webots_ros2_core.tf_publisher import TfPublisher

from webots_ros2_msgs.srv import SetInt

from rosgraph_msgs.msg import Clock

import rclpy
from rclpy.node import Node

try:
    append_webots_python_lib_to_path()
    from controller import Supervisor
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class WebotsNode(Node):
    """
    Extends ROS2 base node to provide integration with Webots.

    Args:
        name (WebotsNode): Webots Robot node.
        args (dict): Arguments passed to ROS2 base node.
        enableTfPublisher (bool): Enable tf2 publisher (deprecated, use `robot_state_publisher` with URDF instead).
        enableJointState (bool): Use `JointStatePublisher` to publish ROS2 messages of type
            [`sensor_msgs/JointState`](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg).
    """

    def __init__(self, name, args=None, enableTfPublisher=False, enableJointState=False):
        super().__init__(name)
        self.declare_parameter('synchronization', False)
        self.declare_parameter('use_joint_state_publisher', False)
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
        self.__device_manager = None
        if enableTfPublisher:
            if self.robot.getSupervisor():
                self.tfPublisher = TfPublisher(self.robot, self)
            else:
                self.get_logger().warn('Impossible to publish transforms because the "supervisor"'
                                       ' field is false.')
        if self.get_parameter('use_joint_state_publisher').value or enableJointState:
            self.jointStatePublisher = JointStatePublisher(self.robot, '', self)

    def step(self, ms):
        if self.get_parameter('use_joint_state_publisher').value:
            self.jointStatePublisher.publish()
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
        self.clockPublisher.publish(msg)

    def timer_callback(self):
        self.step(self.timestep)

    def step_callback(self, request, response):
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
