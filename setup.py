from setuptools import setup

package_name = 'webots_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['launch/launcher.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
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
