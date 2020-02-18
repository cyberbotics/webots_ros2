## Run demo
```
ros2 launch webots_ros2_epuck2 example.launch.py
```

Read data from IR sensors:
```
ros2 topic echo /distance/ps6
```

Set speed:
```
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```