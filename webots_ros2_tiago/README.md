## Run demo
* Open a terminal and go in your R0S2 workspace: `cd PATH/TO/ros2_workspace`
* Build the package: `colcon build --packages-select webots_ros2_tiago`
* Source the package: `. install/setup.bash`
* Launch the demo: `ros2 launch webots_ros2_tiago tiago.launch.py`
* In another terminal,
  * you can publish some velocity commands to control the robot:
    `ros2 topic pub --once /cmd_vel  geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'`
  * you can use `teleop_twist_keyboard` to use the keyboard to control the robot:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
  but make sure you have the `teleop-twist-keyboard` package installed.

### Topics Overview
```
$ ros2 topic list -t
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

### Velocity
```
ros2 topic pub --once /cmd_vel  geometry_msgs/msg/Twist '{
  linear: {
    x: 0.5,
    y: 0.0,
    z: 0.0
  },
  angular: {
    x: 0.0,
    y: 0.0,
    z: 0.0
  }
}'
```

### Visualization
You can visualize the odometry by running RViz2 in parallel with the driver: 
```
ros2 launch webots_ros2_tiago tiago.launch.py rviz:=true
```