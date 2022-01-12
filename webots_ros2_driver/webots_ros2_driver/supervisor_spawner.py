#!/usr/bin/env python3

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
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import String
from webots_ros2_msgs.srv import SetWbURDFRobot

import webots_ros2_driver_webots
from urdf2webots.importer import convert2urdf

# As Driver need the controller library, we extend the path here
# to avoid to load another library named "controller" when loading vehicle library
sys.path.insert(1, os.path.dirname(webots_ros2_driver_webots.__file__))
from controller import Supervisor


class SupervisorSpawner(Node):
    def __init__(self):
        super().__init__('Spawner')

        self.__robot = Supervisor()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        #root_node = self.__robot.getRoot()
        #self.__insertion_robot_place = root_node.getField('children')

        self.create_timer(1.0 / 1000.0, self.__supervisor_step_callback)

        self.get_logger().info('Spawner time step is: '+str(1.0 / 1000.0))
        #self.create_service(SetWbURDFRobot, 'spawn_urdf_robot', self.__spawn_urdf_robot_callback)
        #self.create_subscription(String, 'clean_urdf_robot', self.__clean_urdf_robot_callback, qos_profile_services_default)
        self.get_logger().info('Spawner init !!!!!!!!!!!!!!')

    def __spawn_urdf_robot_callback(self, request, response):
        robot = request.robot

        file_input = robot.urdf_location if robot.urdf_location else ''
        robot_name = robot.name if robot.name else ''
        robot_translation = robot.translation if robot.translation else '0 0 0'
        robot_rotation = robot.rotation if robot.rotation else '0 1 0 0'

        robot_string = convert2urdf(inFile=file_input, robotName=robot_name, initTranslation=robot_translation,
                                    initRotation=robot_rotation)
        self.__insertion_robot_place.importMFNodeFromString(-1, robot_string)

        self.get_logger().info('Spawner has imported the URDF robot "' + str(robot_name) + '"')

        response.success = True
        return response

    def __clean_urdf_robot_callback(self, message):
        robot_node = None
        robotName = message.data

        for ind_node in range(self.__insertion_robot_place.getCount()):
            node = self.__insertion_robot_place.getMFNode(ind_node)
            node_name_field = node.getField("name")
            if node_name_field and node_name_field.getSFString() == robotName:
                robot_node = node
                break

        if robot_node:
            robot_node.remove()
            self.get_logger().info('Spawner has removed the URDF robot "' + str(robotName) + '"')
        else:
            self.get_logger().info('Spawner wanted to remove the URDF robot "' + str(robotName) +
                                   '" but it has not been found.')

    def __supervisor_step_callback(self):
        if self.__robot.step(self.__timestep) < 0:
            self.get_logger().info('Spawner will shut down...')


def main(args=None):
    rclpy.init(args=args)
    supervisorSpawner = SupervisorSpawner()
    rclpy.spin(supervisorSpawner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
