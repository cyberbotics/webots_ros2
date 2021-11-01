"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'webots_ros2_universal_robot'
data_files = [
    ('share/' + package_name + '/worlds', [
        'worlds/universal_robot.wbt',
        'worlds/.universal_robot.wbproj',
        'worlds/armed_robots.wbt',
        'worlds/.armed_robots.wbproj'
    ]),
    ('share/' + package_name + '/resource', [
        'resource/view_robot_dynamic.rviz',
        'resource/webots_ur5e_description.urdf',
        'resource/ros2_control_config.yaml',
        'resource/moveit_ur5e_description.urdf',
        'resource/moveit_controllers.yaml',
        'resource/moveit_ur5e.srdf',
        'resource/moveit_kinematics.yaml',
        'resource/moveit_movegroup.yaml',
        'resource/moveit_visualization.rviz',
        'resource/ros2_control_abb_config.yaml',
        'resource/webots_abb_description.urdf'
    ]),
    ('share/' + package_name + '/launch', [
        'launch/robot_launch.py',
        'launch/multirobot_launch.py',
        'launch/moveit_demo_launch.py',
    ]),
    ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
]

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
            'abb_controller = webots_ros2_universal_robot.abb_controller:main'
        ]
    }
)
