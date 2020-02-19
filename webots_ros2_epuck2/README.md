## Run demo
```
ros2 launch webots_ros2_epuck2 example.launch.py
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
