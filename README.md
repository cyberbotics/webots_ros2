# webots_ros2 ROS2 packages

[![Build Status](https://travis-ci.com/cyberbotics/webots_ros2.svg?branch=master)](https://travis-ci.com/cyberbotics/webots_ros2)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Version](https://img.shields.io/github/v/tag/cyberbotics/webots_ros2?label=version)](http://wiki.ros.org/webots_ros2)


`webots_ros2` is a package that provides the necessary interfaces to simulate a robot in the [Webots](https://cyberbotics.com/) open-source 3D robots simulator.
It integrates with ROS2 using ROS2 messages, services and actions.

[![Webots Video](https://img.youtube.com/vi/O7U3sX_ubGc/0.jpg)](https://www.youtube.com/watch?v=O7U3sX_ubGc)

Some of the key features of Webots include:

- Cross-platform (Windows, Linux and Mac).
- Stable physics engine.
- Reproducibility.
- An efficient rendering engine using [Physically Based Rendering](https://en.wikipedia.org/wiki/Physically_based_rendering) for realistic images.
- A simple and intuitive user interface.
- Wide range of simulated [sensors](https://www.cyberbotics.com/doc/guide/sensors) and [actuators](https://www.cyberbotics.com/doc/guide/actuators) available and ready to work.
- Wide range of [robot models](https://www.cyberbotics.com/doc/guide/robots) available and ready to work.
- Wide range of documented [samples](https://www.cyberbotics.com/doc/guide/sample-webots-applications).

## Getting Started

Install `webots_ros2` package: 
```bash
sudo apt-get install ros-$ROS_DISTRO-webots-ros2
```
And run a demo, for example:
```bash
ros2 launch webots_ros2_demos armed_robots.launch.py
```

> In case you don't have Webots installed, a window will be shown showing you a multiple methods to install Webots.
Click `Automatic` if you want Webots to be automatically downloaded and installed.

Now, you can create your own Webots simulation by following [this tutorial](./webots_ros2_core) or check more demos:
- [webots_ros2_examples](./webots_ros2_examples)
- [webots_ros2_abb](./webots_ros2_abb)
- [webots_ros2_epuck](./webots_ros2_epuck)
- [webots_ros2_universal_robot](./webots_ros2_universal_robot)
- [webots_ros2_tiago](./webots_ros2_tiago)


## Contact us / Technical support
For questions about this package or Webots in general, get in touch with the developers on [Discord](https://discord.gg/fyPuMM4).

## Acknowledgement

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 
