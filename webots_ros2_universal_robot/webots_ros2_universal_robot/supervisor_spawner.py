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

from webots_ros2_msgs.msg import WbURDFRobot
from webots_ros2_msgs.srv import SetWbURDFRobot

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
        self.create_subscription(Bool, 'clean_urdf_robot', self.__clean_urdf_robot_callback, 1)
        self.create_service(SetWbURDFRobot, 'spawn_urdf_robot', self.__spawn_urdf_robot_callback)

    def __spawn_urdf_robot_callback(self, request, response):
        robot = request.robot

        file_input = robot.urdf_location if robot.urdf_location else ''
        robot_name = robot.name if robot.name else ''
        robot_translation = robot.translation if robot.translation else '0 0 0'
        robot_rotation = robot.rotation if robot.rotation else '0 1 0 0'

        robot_string = convert2urdf(inFile=file_input, robotName=robot_name, initTranslation=robot_translation, initRotation=robot_rotation)

        self.__insertion_robot_place.importMFNodeFromString(-1, robot_string)

        response.success = True
        return response

    def __clean_urdf_robot_callback(self, message):
        if message.data:
            self.__insertion_robot_place.removeMF(-1)


    def __supervisor_step_callback(self):
        if self.__robot.step(self.__timestep) < 0:
            self.get_logger().info('supervisor robot step will end')


def main(args=None):
    rclpy.init(args=args)
    supervisorSpawner = SupervisorSpawner()
    rclpy.spin(supervisorSpawner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





'''
class SupervisorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('supervisor_urdf_driver')
        ''''''
        # temp file
        tmp_launch_file = tempfile.NamedTemporaryFile(mode="w", prefix='launch', suffix='.py', delete=False)

        self.__node.get_logger().info("tmp_launch_file file is "+tmp_launch_file.name)

        tmp_launch_file.write()

        tmp_launch_file.close()
        #os.unlink(tmp_launch_file.name)

        self.__node.create_subscription(String, 'robots_list', self.__get_urdf_robot_callback, 1)

        # Refresh urdf
        self.__node.create_subscription(Bool, 'refresh_urdf', self.__refresh_urdf_callback, 1)

        #self.__refresh_urdf_robots_in_webots()

        self.__relative_path_to_launch_file=tmp_launch_file.name
        self.__launch_process = subprocess.Popen(["ros2", "launch", self.__relative_path_to_launch_file], text=True)

        ''''''

        # Add robot to simulation
        self.__node.get_logger().info('supervisor_urdf_driver init will add robot')

        package_dir = get_package_share_directory(PACKAGE_NAME)
        urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')

        robots=[
                {'name': 'UR5e',
                'urdf_location': urdf_path,
                'translation': '0 0 0.6',
                'rotation': '0 0 1 -1.5708',
                },
            ]

        test = self.__robot.getName()

        test2 = self.__robot.getSupervisor()

        self.__node.get_logger().info('supervisor_urdf_driver supervisor is '+ str(test2))

        root_node = self.__robot.getRoot()
        children_field = root_node.getField('children')

        for robot in robots:
            file_input = robot.get('urdf_location')
            robot_name = robot.get('name')
            robot_translation = robot.get('translation')
            robot_rotation = robot.get('rotation')

            if not file_input:
                sys.exit('URDF file not specified (has to be specified with \'urdf_location\': \'path/to/my/robotUrdf.urdf\'')
            if not robot_name:
                sys.stderr.write('Robot name not specified (should be specified if more than one robot is present with \'name\': \'robotName\'\n')

            robot_string = convert2urdf(inFile=file_input, robotName=robot_name, initTranslation=robot_translation, initRotation=robot_rotation)

        children_field.importMFNodeFromString(-1, robot_string)

        self.__pub = self.__node.create_publisher(String, 'test', 1)
        self.__test = 0

    def __get_urdf_robot_callback(self, string):
        self.__node.get_logger().info('URDF robot recieved')

    def __refresh_urdf_callback(self, bool):
        self.__node.get_logger().info('URDF refresh asked')

        if bool.data:
            self.__launch_process.send_signal(signal.SIGINT)
            self.__launch_process.wait(timeout=5)
            self.__launch_process = subprocess.Popen(["ros2", "launch", self.__relative_path_to_launch_file], text=True)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)




        #self.__node.get_logger().info('supervisor_urdf_driver spin !!!')
'''