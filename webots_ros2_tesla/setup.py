"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'webots_ros2_tesla'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/tesla_world.wbt', 'worlds/.tesla_world.wbproj',
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='1.1.2',
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
        'console_scripts': [
            'tesla_driver = webots_ros2_tesla.tesla_driver:main',
            'lane_follower = webots_ros2_tesla.lane_follower:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
