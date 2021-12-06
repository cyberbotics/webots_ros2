"""webots_ros2 package setup file."""

import os

from setuptools import setup


package_name = 'webots_ros2_urdf_converter'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/world.wbt',
]))
data_files.append(('share/' + package_name, ['package.xml']))

robot_model_folder = 'resource/kuka_lbr_iiwa_support'

for (path, _, filenames) in os.walk(robot_model_folder):
    for filename in filenames:
        data_files.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))


setup(
    name=package_name,
    version='1.2.0',
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
    description='Tesla ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
