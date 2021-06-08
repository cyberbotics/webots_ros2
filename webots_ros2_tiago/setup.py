"""webots_ros2 package setup file."""

from setuptools import setup

package_name = 'webots_ros2_tiago'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/tiago.launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/ros_tiago.wbt', 'worlds/.ros_tiago.wbproj',
    'worlds/tiago++_example.wbt', 'worlds/.tiago++_example.wbproj'
]))
data_files.append(('share/' + package_name + '/resource',
                   ['resource/odometry.rviz', 'resource/tiago.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))


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
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples', 'TIAGo'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TIAGo robots ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
