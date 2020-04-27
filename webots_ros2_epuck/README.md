# ROS2 Driver for E-Puck Robot in Webots
This package adds ROS2 support for [e-puck](https://www.gctronic.com/doc/index.php/e-puck2) simulated robot in [Webots](https://cyberbotics.com/).
Since ROS2 driver for the physical ([`epuck_ros2`](https://github.com/cyberbotics/epuck_ros2)) and simulated ([`webots_ros2_epuck`](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_epuck)) robot share the same API you can find more instructions [here](./EPUCK_ROS2.md). 

![Mapping process](./assets/mapping.gif) 

## Launching the Simulation
There is a launch file available that starts Webots simulation and e-puck ROS2 driver.
``` 
ros2 launch webots_ros2_epuck robot_launch.py
```
