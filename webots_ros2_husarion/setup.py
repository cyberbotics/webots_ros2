from setuptools import setup

package_name = 'webots_ros2_husarion'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/rosbot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/rosbot_xl_launch.py']))
data_files.append(('share/' + package_name + '/resource', ['resource/rosbot_controllers.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/rosbot_xl_controllers.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/ekf.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/laser_filter.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/rosbot_webots.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/rosbot_links_remappings.yaml']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/rosbot.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/rosbot_xl.wbt']))
data_files.append(('share/' + package_name + '/worlds/meshes', ['worlds/meshes/husarion_world.dae']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='2025.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jakub Delicat',
    author_email='jakub.delicat@husarion.com',
    maintainer='Husarion',
    maintainer_email='support@husarion.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples', 'ROSbot', 'ROSbot 2R', 'ROSbot XL', 'Husarion'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Husarion ROSbot 2R and XL robots ROS2 interface for Webots.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
