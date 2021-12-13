"""webots_ros2 package setup file."""

from setuptools import setup
import os


package_name = 'webots_ros2_universal_robot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
    'launch/multirobot_launch.py',
    'launch/moveit_demo_launch.py',
    'launch/test_launch.py',
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/universal_robot.wbt',
    'worlds/.universal_robot.wbproj',
    'worlds/armed_robots.wbt',
    'worlds/.armed_robots.wbproj'
]))
data_files.append(('share/' + package_name, [
    'package.xml'
]))


robot_model_folder = 'resource'

for (path, _, filenames) in os.walk(robot_model_folder):
    for filename in filenames:
        data_files.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))


setup(
    name=package_name,
    version='1.2.0',
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
            'abb_controller = webots_ros2_universal_robot.abb_controller:main',
            'supervisor_driver = webots_ros2_universal_robot.supervisor_driver:main',
            'supervisor_node = webots_ros2_universal_robot.supervisor_node:main'
        ]
    }
)
