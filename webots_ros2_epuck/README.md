# webots_ros2_epuck
This package adds ROS2 support for e-puck simulated robot in Webots.

## Launching the Simulation
There is a launch file available that starts Webots simulation and e-puck ROS2 driver.
``` 
ros2 launch webots_ros2_epuck example_launch.py
```

## Interact with the Robot
Since ROS2 driver for physical (`epuck_ros2`) and simulated (`webots_ros2_epuck`) robot share the same API you find more instructions [here](./EPUCK_ROS2.md). 