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

"""ROS2 Mavic 2 Pro driver."""


import rclpy
from geometry_msgs.msg import Twist

from std_msgs.msg import String, Bool


import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

import subprocess
import signal

import tempfile

import sys


from urdf2webots.importer import convert2urdf





PACKAGE_NAME = 'webots_ros2_universal_robot'

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')
    robot_description = pathlib.Path(urdf_path).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster'] + controller_manager_timeout,
    )

    universal_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5e'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            ros2_control_params
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Use webots.getDriversList() to get all the drivers node, the basis of the
    # webots_ros2 interface.
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        robot_state_publisher,
        universal_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=universal_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])


class SupervisorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('supervisor_urdf_driver')

        # temp file
        tmp_launch_file = tempfile.NamedTemporaryFile(mode="w", prefix='launch', suffix='.py', delete=False)

        self.__node.get_logger().info("tmp_launch_file file is "+tmp_launch_file.name)

        tmp_launch_file.write("""
import os
import pathlib
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')
    robot_description = pathlib.Path(urdf_path).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster'] + controller_manager_timeout,
    )

    universal_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5e'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            ros2_control_params
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Use webots.getDriversList() to get all the drivers node, the basis of the
    # webots_ros2 interface.
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        robot_state_publisher,
        universal_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=universal_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
""")

        tmp_launch_file.close()
        #os.unlink(tmp_launch_file.name)

        self.__node.create_subscription(String, 'robots_list', self.__get_urdf_robot_callback, 1)

        # Refresh urdf
        self.__node.create_subscription(Bool, 'refresh_urdf', self.__refresh_urdf_callback, 1)

        #self.__refresh_urdf_robots_in_webots()

        self.__relative_path_to_launch_file=tmp_launch_file.name
        self.__launch_process = subprocess.Popen(["ros2", "launch", self.__relative_path_to_launch_file], text=True)

        self.__node.get_logger().info('supervisor_urdf_driver init finished')

        print('supervisor_urdf_driver init finished with print !')


        # Add robot to simulation
        '''
        package_dir = get_package_share_directory(PACKAGE_NAME)
        urdf_path = os.path.join(package_dir, 'resource', 'ur_description', 'urdf', 'ur5e.urdf')

        robots=[
                {'name': 'UR5e',
                'urdf_location': urdf_path,
                'translation': '0 0 0.6',
                'rotation': '0 0 1 -1.5708',
                },
            ]

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

            robot_file = convert2urdf(inFile=file_input, robotName=robot_name, initTranslation=robot_translation, initRotation=robot_rotation)

        children_field.importMFNode(-1, robot_file)
        '''

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

        self.__test += 1

        if self.__test > 20:
            msg = String()
            msg.data = str(self.__test)

            self.__pub.publish(msg)
            self.__test = 0




        #self.__node.get_logger().info('supervisor_urdf_driver spin !!!')
