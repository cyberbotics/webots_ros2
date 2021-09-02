<<<<<<< HEAD
#!/usr/bin/env bash

if ["${ROS_DISTRO}" = "foxy"]; then
    apt install ros-foxy-turtlebot3-cartographer
    apt install ros-foxy-turtlebot3-navigation2
fi

# below comes from a merge, get rid of if not necessary
# 75f075f44ec467c2a685fc20cc3d480d153f57fd