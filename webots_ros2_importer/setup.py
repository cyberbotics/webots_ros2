"""webots_ros2_importer package setup file."""

from setuptools import setup

package_name = 'webots_ros2_importer'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='2023.1.3',
    packages=[package_name, package_name + '.urdf2webots.urdf2webots'],
    data_files=data_files,
    install_requires=[
        'setuptools'
    ],
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
    description='This package allows to convert URDF and XACRO files into Webots PROTO files.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urdf2proto = webots_ros2_importer.urdf2proto:main',
            'xacro2proto = webots_ros2_importer.xacro2proto:main'
        ],
    }
)
