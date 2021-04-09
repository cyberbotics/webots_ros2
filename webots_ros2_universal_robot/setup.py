"""webots_ros2 package setup file."""

import os
import fnmatch

from setuptools import setup

package_name = 'webots_ros2_universal_robot'
worlds = [
    'worlds/universal_robot_multiple.wbt',
    'worlds/universal_robot_rviz.wbt',
    'worlds/universal_robot.wbt',
    'worlds/universal_robot_lidar.wbt',
    'worlds/.universal_robot_multiple.wbproj',
    'worlds/.universal_robot_rviz.wbproj',
    'worlds/.universal_robot.wbproj',
    'worlds/.universal_robot_lidar.wbproj'
]
launchers = [
    'launch/universal_robot.launch.py',
    'launch/universal_robot_multiple.launch.py',
    'launch/universal_robot_rviz.launch.py',
    'launch/universal_robot_rviz_dynamic.launch.py',
    'launch/universal_robot_moveit2.launch.py',
    'launch/universal_robot_moveit2.rviz'
]
config = [
    'config/controllers.yaml',
    'config/ur5e.srdf',
    'config/kinematics.yaml'
]
textures = []
for rootPath, dirNames, fileNames in os.walk('worlds/textures'):
    for fileName in fnmatch.filter(fileNames, '*.jpg'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        textures.append(filePath)

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', launchers))
data_files.append(('share/' + package_name + '/config', config))
data_files.append(('share/' + package_name + '/worlds', worlds))
data_files.append(('share/' + package_name + '/worlds/textures', textures))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', ['resource/view_robot_dynamic.rviz']))

setup(
    name=package_name,
    version='1.0.6',
    packages=[],
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
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
