from setuptools import find_packages, setup

package_name = 'webots_ros2_crazyflie'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/crazyflie_apartment.wbt', 'worlds/.crazyflie_apartment.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/crazyflie_webots.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='2025.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Kimberly McGuire (Bitcraze AB)',
    maintainer_email='kimberly@bitcraze.io',
    description='ROS2 package for Crazyflie webots simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_driver = webots_ros2_crazyflie.crazyflie_driver:main',
        ],
    },
)
