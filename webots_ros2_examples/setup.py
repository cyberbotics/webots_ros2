"""webots_ros2 package setup file."""

from glob import glob
from setuptools import setup


package_name = 'webots_ros2_examples'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/example.launch.py']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/khepera4.yaml',
]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/Khepera4_enu.proto'
]))
data_files.append(('share/' + package_name + '/protos/textures', glob('protos/textures/*')))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/ros_example.wbt', 'worlds/.ros_example.wbproj',
    'worlds/khepera4_example.wbt', 'worlds/.khepera4_example.wbproj'
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
    description='Minimal example showing how to control a robot with ROS2 in Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_controller = webots_ros2_examples.example_controller:main',
            'khepera_driver = webots_ros2_examples.khepera_driver:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
