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

"""ROS2 Webots URDF Robots spawner."""


import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

import subprocess
import signal

import tempfile

import sys


from urdf2webots.importer import convert2urdf

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

import webots_ros2_driver_webots

from webots_ros2_msgs.srv import SetWbURDFRobot
from webots_ros2_msgs.srv import ClearRobot

# As Driver need the controller library, we extend the path here
# to avoid to load another library named "controller" when loading vehicle library
sys.path.insert(1, os.path.dirname(webots_ros2_driver_webots.__file__))
from controller import Supervisor





class SupervisorSpawner(Node):
    def __init__(self):
        super().__init__('super_spawn')

        self.__robot = Supervisor()
        self.__timestep = int(self.__robot.getBasicTimeStep())
        root_node = self.__robot.getRoot()
        self.__insertion_robot_place = root_node.getField('children')

        self.create_timer(self.__timestep / 1000 , self.__supervisor_step_callback)
        self.create_service(SetWbURDFRobot, 'spawn_urdf_robot', self.__spawn_urdf_robot_callback)
        self.create_service(ClearRobot, 'clean_urdf_robot', self.__clean_urdf_robot_callback)

    def __spawn_urdf_robot_callback(self, request, response):
        robot = request.robot

        file_input = robot.urdf_location if robot.urdf_location else ''
        robot_name = robot.name if robot.name else ''
        robot_translation = robot.translation if robot.translation else '0 0 0'
        robot_rotation = robot.rotation if robot.rotation else '0 1 0 0'

        self.get_logger().info('supervisor import urdf robot '+str(robot))

        robot_string = convert2urdf(inFile=file_input, robotName=robot_name, initTranslation=robot_translation, initRotation=robot_rotation)
        self.__insertion_robot_place.importMFNodeFromString(-1, robot_string)

        response.success = True
        return response

    def __clean_urdf_robot_callback(self, request, response):
        self.get_logger().info('supervisor clean urdf of robot: '+str(request.name))

        self.__insertion_robot_place.removeMF(-1)

        response.success = True
        return response


    def __supervisor_step_callback(self):
        #self.get_logger().info('supervisor robot step ')
        if self.__robot.step(self.__timestep) < 0:
            self.get_logger().info('supervisor robot step return -1 !!!')
        #self.get_logger().info('supervisor robot step end')


def main(args=None):
    rclpy.init(args=args)
    supervisorSpawner = SupervisorSpawner()
    rclpy.spin(supervisorSpawner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''
"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.


'''
