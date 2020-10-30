from glob import glob
from setuptools import setup

package_name = 'webots_ros2_tutorials'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name, [
    'launch/line_following_launch.py'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/custom_line_follower.wbt'
]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/Robot_sense.proto'
]))

data_files.append(('share/' + package_name + '/protos/icons', glob('protos/icons/*')))
data_files.append(('share/' + package_name + '/worlds/textures', glob('worlds/textures/*')))
data_files.append(('share/' + package_name + '/protos/textures', glob('protos/textures/*')))

data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    keywords=['ROS2', 'Webots', 'Soft_Illusion', 'Tutorials', 'Youtube', 'Simulation'],
    maintainer='Soft_illusion',
    maintainer_email='harsh.b.kakashaniya@gmail.com',
    description='Projects for videos for webots ros2 tutorial series on youtube',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Sub_tof = webots_ros2_tutorials.Sub_tof:main',
            'enable_robot = webots_ros2_tutorials.slave:main',
            'line_follower = webots_ros2_tutorials.master:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
