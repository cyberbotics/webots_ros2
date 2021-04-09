"""webots_ros2 package setup file."""

from glob import glob
from setuptools import setup


package_name = 'webots_ros2_turtlebot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot3_burger_example_map.pgm',
    'resource/turtlebot3_burger_example_map.yaml'
]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/TurtleBot3Burger_enu.proto'
]))
data_files.append(('share/' + package_name + '/protos/textures', glob('protos/textures/*')))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/turtlebot3_burger_example.wbt', 'worlds/.turtlebot3_burger_example.wbproj',
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='1.0.6',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TurtleBot3 Burger robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_driver = webots_ros2_turtlebot.turtlebot_driver:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
