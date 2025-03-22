from setuptools import setup

package_name = 'webots_ros2_tiago'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
    'launch/robot_bringup_launch.py'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/tiago_webots.urdf',
    'resource/tiago_bringup_webots.urdf',
    'resource/ros2_control.yml',
    'resource/ros2_control_bringup.yml',
    'resource/nav2_params.yaml',
    'resource/nav2_params_iron.yaml',
    'resource/default.rviz',
    'resource/default_bringup.rviz',
    'resource/map.pgm',
    'resource/map.yaml',
    'resource/cartographer.lua',
    'resource/slam_toolbox_params.yaml',
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/default.wbt',
    'worlds/.default.wbproj',
    'worlds/default_bringup.wbt',
    'worlds/.default_bringup.wbproj'
]))

setup(
    name=package_name,
    version='2025.0.0',
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
