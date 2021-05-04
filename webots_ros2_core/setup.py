"""webots_ros2 package setup file."""

from setuptools import setup

package_name = 'webots_ros2_core'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))


setup(
    name=package_name,
    version='1.0.6',
    packages=[package_name, package_name + '.devices', package_name + '.math', package_name + '.webots'],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Core interface between Webots and ROS2.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_differential_drive_node = webots_ros2_core.webots_differential_drive_node:main',
            'webots_robotic_arm_node = webots_ros2_core.webots_robotic_arm_node:main',
            'webots_node = webots_ros2_core.webots_node:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
