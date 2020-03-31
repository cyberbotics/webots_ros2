"""webots_ros2_epuck package setup file."""

from setuptools import setup

package_name = 'webots_ros2_epuck'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/example_launch.py']))
data_files.append(('share/' + package_name + '/worlds',
                   ['worlds/epuck_world.wbt', 'worlds/.epuck_world.wbproj']))
data_files.append(('share/' + package_name + '/resource',
                   ['resource/all.rviz']))
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
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='E-puck2 driver for Webots simulated robot',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = webots_ros2_epuck.driver:main',
            'drive_calibrator = webots_ros2_epuck.drive_calibrator:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
