from setuptools import setup

import os

package_name = 'webots_ros2'
data_files = []

# Add Webots in the package
# for root, directories, files in os.walk(os.environ['WEBOTS_HOME']):
#     for f in files:
#        source = os.path.relpath(os.path.join(root, f))
#        target = root.replace(os.environ['WEBOTS_HOME'], 'share/' + package_name + '/webots')
#        data_files.append((target, [source]))

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/example.launch.py']))
data_files.append(('share/' + package_name, ['launch/universal_robot.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/ros_example.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/universal_robot.wbt']))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Interface between Webots and ROS2.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_controller = webots_ros2.example_controller:main',
            'webots_launcher = webots_ros2.webots_launcher:main',
            'universal_robot = webots_ros2.universal_robot:main',
        ],
        'launch.frontend.launch_extension': [
            'launch_ros = launch_ros',
        ],
    },
)
