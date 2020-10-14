# Installation

## From Sources

### Webots Software
Webots is a prerequisite to use the `webots_ros2` package.
It can be downloaded from the [Github repository](https://github.com/cyberbotics/webots/releases/latest) or build from [the sources](https://github.com/cyberbotics/webots).
The installation is straightforward, but if need the installation instructions can be found [here](https://www.cyberbotics.com/doc/guide/installing-webots).

Alternatively, you can also get let `webots_ros2` download Webots automatically by click `Automatic` in a window that appears when you lunch an example.

#### Multiple Installations of Webots
If you have more than one installation of Webots, ROS2 will look for Webots at the following locations (in this order):
1. If the `ROS2_WEBOTS_HOME` environment variable is set, ROS2 will use the Webots in this folder.
3. If the `WEBOTS_HOME` environment variable is set, ROS2 will use the Webots in this folder.
4. If none of the previous point is set/installed ROS2 will look for Webots in the default installation paths (e.g. `/usr/local/webots`).
5. If Webots couldn't be found, `webots_ros2` will show a window and offer automatic Webots installation.

### webots_ros2 Package
The following instructions assume that a [ROS2 workspace](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial) is already available.

#### Linux
```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Retrieve the sources
cd /path/to/ros2_ws/src
git clone --recurse-submodules -b $ROS_DISTRO https://github.com/cyberbotics/webots_ros2.git
cd ..

# Check dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Building packages
colcon build

# Source this workspace (careful when also sourcing others)
source install/setup.bash
```

#### Windows
```bash
call C:\dev\ros2\local_setup.bat

# Install Webots from: https://www.cyberbotics.com/download

# Retrieve the sources
cd /path/to/catkin_ws/src
git clone --recurse-submodules -b $ROS_DISTRO https://github.com/cyberbotics/webots_ros2.git
cd ..

# Check dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Building packages
colcon build

# Source this workspace (careful when also sourcing others)
call install\setup.bat
```

The packages can now be used as regular ROS packages.

Refer to the [colcon tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial) for more information on building ROS2 workspace with colcon.