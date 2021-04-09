"""ur_e_description package setup file."""

import os
import fnmatch

from setuptools import setup

package_name = 'webots_ros2_ur_e_description'
data_files = []

# Add STL files
for rootPath, dirNames, fileNames in os.walk('meshes'):
    for fileName in fnmatch.filter(fileNames, '*.stl'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Add DAE files
for rootPath, dirNames, fileNames in os.walk('meshes'):
    for fileName in fnmatch.filter(fileNames, '*.dae'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Add URDF files
for rootPath, dirNames, fileNames in os.walk('urdf'):
    for fileName in fnmatch.filter(fileNames, '*.urdf'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Other files
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/ur5e_state_publisher.launch.py']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/view_robot.rviz']))
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
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Universal Robots', 'RVIZ'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Universal Robot description for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
