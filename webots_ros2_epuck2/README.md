## Run demo
```
ros2 launch webots_ros2_epuck2 example.launch.py
```
and you can use `teleop_twist_keyboard` in another window to control the robot:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
but make sure you have `ros-eloquent-teleop-twist-keyboard` installed.

### Topics Overview
```
$ ros2 topic list -t
/camera/camera_info [sensor_msgs/msg/CameraInfo]
/camera/image_raw [sensor_msgs/msg/Image]
/cmd_vel [geometry_msgs/msg/Twist]
/distance/ps0 [sensor_msgs/msg/Range]
/distance/ps1 [sensor_msgs/msg/Range]
/distance/ps2 [sensor_msgs/msg/Range]
/distance/ps3 [sensor_msgs/msg/Range]
/distance/ps4 [sensor_msgs/msg/Range]
/distance/ps5 [sensor_msgs/msg/Range]
/distance/ps6 [sensor_msgs/msg/Range]
/distance/ps7 [sensor_msgs/msg/Range]
```

### Read IR Sensors
```
ros2 topic echo /distance/ps6
```

### Set Velocity
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

### Stream Camera Images
Run `rqt`, navigate to `Plugins > Visualization > Image View` and for topic choose `/camera/image_raw`. Note that the image encoding is BGRA.

> Make sure your QT5 Plugins are properly configured:  
> https://askubuntu.com/questions/308128/failed-to-load-platform-plugin-xcb-while-launching-qt5-app-on-linux-without
> ```
> sudo ln -sf /usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/ /usr/bin/
> ```

### Control LEDs
```
ros2 service call /set_led0 webots_ros2_msgs/srv/SetInt "{ value: 1 }"
```