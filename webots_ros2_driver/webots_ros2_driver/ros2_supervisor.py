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
import vehicle
import controller
import webots_ros2_importer
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
sys.path.insert(1, os.path.join(os.path.dirname(webots_ros2_importer.__file__), 'urdf2webots'))
from urdf2webots.importer import convertUrdfFile, convertUrdfContent
from webots_ros2_msgs.srv import SpawnUrdfRobot

# As Ros2Supervisor needs the controller library, we extend the path here
# to avoid to load another library named "controller" or "vehicle".
sys.path.insert(1, os.path.dirname(vehicle.__file__))
sys.path.insert(1, os.path.dirname(controller.__file__))
from controller import Supervisor


class Ros2Supervisor(Node):
    def __init__(self):
        super().__init__('Ros2Supervisor')

        self.__robot = Supervisor()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # /clock topic
        self.create_timer(1 / 1000, self.__supervisor_step_callback)
        self.__clock_publisher = self.create_publisher(Clock, 'clock', 10)

        # Spawn URDF robots
        root_node = self.__robot.getRoot()
        self.__insertion_robot_place = root_node.getField('children')
        self.__urdf_robots_list=[]
        self.create_service(SpawnUrdfRobot, 'spawn_urdf_robot', self.__spawn_urdf_robot_callback)
        self.create_subscription(String, 'remove_urdf_robot', self.__remove_urdf_robot_callback, qos_profile_services_default)

    def __spawn_urdf_robot_callback(self, request, response):
        robot = request.robot

        robot_name = robot.name if robot.name else ''

        if robot_name == '':
            self.get_logger().info('Ros2Supervisor cannot import an unnamed URDF robot. Please specifiy it with name="" in the URDFSpawner object.')
            response.success = False
            return response
        if robot_name in self.__urdf_robots_list:
            self.get_logger().info('The URDF robot name "' + str(robot_name) + '" is already used by another robot! Please specifiy a unique name.')
            response.success = False
            return response

        robot_translation = robot.translation if robot.translation else '0 0 0'
        robot_rotation = robot.rotation if robot.rotation else '0 0 1 0'
        normal = robot.normal if robot.normal else False
        box_collision = robot.box_collision if robot.box_collision else False
        init_pos = robot.init_pos if robot.init_pos else None

        # Choose the conversion according to the input
        if robot.urdf_path:
            robot_string = convertUrdfFile(input=robot.urdf_path, robotName=robot_name, normal=normal,
                                    boxCollision=box_collision, initTranslation=robot_translation, initRotation=robot_rotation,
                                    initPos=init_pos)
        elif robot.robot_description:
            relative_path_prefix = robot.relative_path_prefix if robot.relative_path_prefix else None
            robot_string = convertUrdfContent(input=robot.robot_description, robotName=robot_name, normal=normal,
                                        boxCollision=box_collision, initTranslation=robot_translation, initRotation=robot_rotation,
                                        initPos=init_pos, relativePathPrefix=relative_path_prefix)
        else:
            self.get_logger().info('Ros2Supervisor can not import a URDF file without a specified "urdf_path" or "robot_description" in the URDFSpawner object.')
            response.success = False
            return response

        self.__insertion_robot_place.importMFNodeFromString(-1, robot_string)
        self.get_logger().info('Ros2Supervisor has imported the URDF robot named "' + str(robot_name) + '".')
        self.__urdf_robots_list.append(robot_name)
        response.success = True
        return response

    def __remove_urdf_robot_callback(self, message):
        robotName = message.data

        if robotName in self.__urdf_robots_list:
            robot_node = None

            for id_node in range(self.__insertion_robot_place.getCount()):
                node = self.__insertion_robot_place.getMFNode(id_node)
                node_name_field = node.getField('name')
                if node_name_field and node_name_field.getSFString() == robotName:
                    robot_node = node
                    break

            if robot_node:
                robot_node.remove()
                self.__urdf_robots_list.remove(robotName)
                self.get_logger().info('Ros2Supervisor has removed the URDF robot named "' + str(robotName) + '".')
            else:
                self.get_logger().info('Ros2Supervisor wanted to remove the URDF robot named "' + str(robotName) +
                                    '" but this robot has not been found in the simulation world.')

    def __supervisor_step_callback(self):
        if self.__robot.step(self.__timestep) < 0:
            self.get_logger().info('Ros2Supervisor is shutting down...')
        else:
            clock_message = Clock()
            clock_message.clock = Time(seconds=self.__robot.getTime()).to_msg()
            self.__clock_publisher.publish(clock_message)


def main(args=None):
    rclpy.init(args=args)
    ros_2_supervisor = Ros2Supervisor()
    rclpy.spin(ros_2_supervisor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
