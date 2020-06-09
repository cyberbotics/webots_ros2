# webots_ros2_core

This package contains essential building blocks for running Webots simulation, such as Webots launcher, ROS2 wrappers for Webots devices and other relevant utils.

## Creating ROS2 Driver for Webots
Word "driver" in the context of ROS is usually considered to be a ROS node which has a tight interaction with a robot (physical or simulated).
Therefore, in the further text, we will explain how to create ROS2 node that tightly interacts with the simulated robot in Webots.

### Examples
This Github repository contains a few good examples that you can use as the starting point:
- `webots_ros2_example` includes a very simple controller for Thymio (differential driver robot).
- `webots_ros2_tiago` is another differential drive robot, but here `WebotsDifferentialDriveNode` class from `webots_ros2_core` is utilized to simplify differential drive implementation.
- `webots_ros2_epuck` is again differential drive robot that creates ROS services and topics for almost all available sensors and actuators.
This example also contains a [list of instructions](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/EPUCK_ROS2.md) that explains how the simulation can be used in combination with different ROS2 packages like RViz and Navigation2.
Also, you will this example useful if you are planning later to go to the real robot as we support [ROS2 driver for the real robot](https://github.com/cyberbotics/epuck_ros2) as well.

### Universal Launcher
In `webots_ros2_core` package we provide launcher that supposed to automatically create ROS2 services and topics based on Webots' robot description (popularly called [ROSification](https://roscon.ros.org/2013/wp-content/uploads/2013/06/ROSCon2013_rosify_robot.pdf)). It is enough to provide path to Webots world file with the robot inside, for example: 
```
ros2 launch webots_ros2_core robot_launch.py world:=$(ros2 pkg prefix webots_ros2_universal_robot --share)/worlds/universal_robot_rviz.wbt
```
This command will run Webots with [UR5](https://www.universal-robots.com/products/ur5-robot/) and create necessary topics to visualize link positions in RViz.
