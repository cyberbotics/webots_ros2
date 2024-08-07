"""webots_ros2 package setup file."""

from setuptools import setup
import os


package_name = 'webots_ros2_universal_robot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch/robot_nodes_launch.py',
    'launch/robot_launch/robot_world_launch.py',
    'launch/robot_launch/robot_moveit_nodes_launch.py',
    'launch/multirobot_launch.py',
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/universal_robot.wbt',
    'worlds/.universal_robot.wbproj',
    'worlds/robotic_arms.wbt',
    'worlds/.robotic_arms.wbproj'
]))
data_files.append(('share/' + package_name, [
    'package.xml'
]))

for (path, _, sub_folder) in os.walk('resource'):
    for filename in sub_folder:
        if filename != package_name:  # do not add the empty 'package_name' file
            data_files.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))

setup(
    name=package_name,
    version='2023.1.3',
    packages=['webots_ros2_universal_robot'],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Universal Robots'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Universal Robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
            'ur5e_controller = webots_ros2_universal_robot.ur5e_controller:main',
            'abb_controller = webots_ros2_universal_robot.abb_controller:main'
        ]
    }
)
