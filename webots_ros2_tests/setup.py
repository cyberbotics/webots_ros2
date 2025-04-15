from setuptools import setup

package_name = 'webots_ros2_tests'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/worlds', ['worlds/driver_test.wbt', 'worlds/.driver_test.wbproj', 'worlds/connector_and_vacuum_gripper_test.wbt', 'worlds/.connector_and_vacuum_gripper_test.wbproj']),
    ('share/' + package_name + '/resource', ['resource/driver_test.urdf', 'resource/connector_and_vacuum_gripper_test.urdf', 'resource/object_position.urdf.xacro', 'resource/connector_and_vacuum_gripper_test_ros2_control.yml'])
]

setup(
    name=package_name,
    version='2025.0.0',
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
    description='System tests for `webots_ros2` packages',
    license='Apache License, Version 2.0',
    tests_require=['pytest']
)
