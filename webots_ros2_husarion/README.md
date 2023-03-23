# ROSbot 2R, XL

This package provides a ROS2 interface example for the simulated ROSbot 2R, XL robot in Webots.

# Dependencies
```
sudo apt install python3-vcs python3-rosdep
```

# Build

### Clone the repository
```
git clone https://github.com/cyberbotics/webots_ros2 src/webots_ros2
```

### Initialize submodules
```
cd src/webots_ros2
git submodules update --init
cd ../../
```

### Download dependencies and remove unnecessary
```
vcs import src/webots_ros2/webots_ros2_husarion < src/webots_ros2/webots_ros2_husarion/webots_ros2_husarion.repos
find src/webots_ros2/webots_ros2_husarion/rosbot* -maxdepth 1 -type d !  \( -name "*_description"  -o -name "*_ros" \) -exec rm -r {} \;

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --ignore-src --from-path src/webots_ros2/ -y --rosdistro $ROS_DISTRO
```

### Build packages
```
colcon build
```

# Run
```
source install/setup.bash
```
### ROSbot 2R
```
ROBOT_NAME=rosbot ros2 launch webots_ros2_husarion robot_launch.py robot_name:=${ROBOT_NAME}
```

### ROSbot XL
```
ROBOT_NAME=rosbot_xl ros2 launch webots_ros2_husarion robot_launch.py robot_name:=${ROBOT_NAME}
```