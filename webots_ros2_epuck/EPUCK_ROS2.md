# ROS2 for E-Puck
Here you will find instructions on how to use the e-puck ROS2 API. 
This documentation is common for the physical ([`epuck_ros2`](https://github.com/cyberbotics/epuck_ros2)) and simulated ([`webots_ros2_epuck`](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_epuck)) robot.

## Getting Started
Please make sure that you have the robot driver running.
This will ensure that access to sensors and actuators is exposed through the ROS2 API.

### Simulated Robot
The launch file starts Webots and ROS2 driver:
```
ros2 launch webots_ros2_epuck robot_launch.py
```

For the convience, you can also launch `robot_with_tools_launch.py` which includes `robot_launch.py` and `robot_tools_launch.py`, e.g.:
```
ros2 launch webots_ros2_epuck robot_with_tools_launch.py rviz:=true
```
This launch file has the same parameters as `robot_tools_launch.py`.

### Physical Robot
This launch file starts e-puck driver for physical robot:
```
ros2 launch epuck_ros2 robot_launch.py
```

## Tutorials
When you have the ROS2 driver running you can proceed with the tutorials.

### Infra-red, Light Sensors, and LEDs
![e-puck2 camera and infrared sensors](./assets/sensors_and_leds.png)  
E-puck2 has 8 infra-red sensors (named as `ps0-7`) all of which are mapped to the same name ROS2 topics of type [sensor_msgs/Range](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg).
Therefore, you can obtain a distance from a sensor as follows:
``` 
ros2 topic echo /ps1
```
Besides infrared sensors, e-puck2 is upgraded with long-range ToF sensor positioned just above the camera.
Data from this sensor is also exposed through the [sensor_msgs/Range](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg) topic with name `tof`.

All distance sensors are combined to create [sensor_msgs/LaserScan](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserScan.msg) so you can use it directly in SLAM packages.
You can test it as:
```
ros2 topic echo /scan
```

The same infra-red sensors act as light sensors.
In the ROS2 driver, data from the sensors is published as [sensor_msgs/Illuminance](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Illuminance.msg) message (unit is lux) and you can subscribe to it as follows:
``` 
ros2 topic echo /ls1
```
Notice in the image above, there are 8 LEDs as well-positioned around the robot.
LEDs `led0`, `led2`, `led4` and `led6` can be only turned on or off, while LEDs `led1`, `led3`, `led5` and `led7` have controllable RGB components.
Therefore, in the case of binary LEDs, you can test them as:
``` 
ros2 topic pub /led0 std_msgs/Bool '{ "data": true }'
```
and RGB as:
```
ros2 topic pub /led1 std_msgs/Int32 '{ "data": 0xFF0000 }'
```
where 3 lower bytes of Int32 represent 3 bytes of R, G and B components.

### Velocity Control
Standard [geometry_msgs/Twist](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg) topic with name `/cmd_vel` is exposed for velocity control.
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
> Note that only `linear.x` and `angular.z` are considered as e-puck2 is differential wheeled robot.

### Odometry
Standard ROS2 messages [nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg) are used to publish odometry data
You can subscribe to it with:
``` 
ros2 topic echo /odom
```
In case you are not interested in covariance matrices, you can use `--no-arr` parameter to hide arrays:
```
ros2 topic echo --no-arr /odom
```

> Note that ROS uses [REP](https://www.ros.org/reps/rep-0103.html) convention (x-forward, y-left and z-up) while Webots inherited convention from [VRML](https://en.wikipedia.org/wiki/VRML) (x-right, y-up, z-backward).
> Therefore, you will see translations in Webots as following <img src ="https://render.githubusercontent.com/render/math?math=x_{Webots} = -y_{ROS}" />, <img src ="https://render.githubusercontent.com/render/math?math=y_{Webots} = z_{ROS}" /> and <img src ="https://render.githubusercontent.com/render/math?math=z_{Webots} = -x_{ROS}" />.

You can also visualise odometry in `rviz` :

``` 
ros2 launch webots_ros2_epuck robot_tools_launch.py rviz:=true
```

### Camera
Camera data and details are described through `image_raw` (type [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)) and `camera_info` (type [sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)) topics.
Compared to the physical robot driver there is no [sensor_msgs/CompressedImage](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CompressedImage.msg) since the images are not meant to be transfer thourgh a network.

You can run `rqt` , navigate to `Plugins > Visualization > Image View` and for topic choose `/image_raw`.
Note that the image encoding is BGRA.

### IMU
There are 3D accelerometer and 3D gyro hardware on e-puck2.
You can access to this data through `imu` topic (type [sensor_msgs/Imu](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)), e.g.:
```
ros2 topic echo --no-arr /imu
```

### Ground Sensors
Ground sensors come as an [optional module](http://www.e-puck.org/index.php?option=com_content&view=article&id=17&Itemid=18) for e-puck2.
If the module is present, the ROS2 driver will automatically detect it and publish data.
You can test them as:
``` 
ros2 topic echo /gs1
```
To put the ground sensor module, select `groundSensorsSlot` in `e-puck2` robot tree, click `+` button and find `E-puckGroundSensors` (check out [this image](./assets/ground_sensors_webots.png)).

### Transformations
Dynamic transformations are only used for the odometry and you can show it as:
```
ros2 topic echo tf
```

All other transformations are static and they are exposed as latched topics, so you can show them with the following command:
```
ros2 topic echo --qos-profile services_default --qos-durability transient_local tf_static
```

For general access to transformations you can use `tf2_monitor`:
```
ros2 run tf2_ros tf2_monitor
```
or if you want to read transformation between arbitrary two coordinate frames in a tree:
```
ros2 run tf2_ros tf2_echo odom map
```

### Navigation
ROS2 Navigation2 stack (see [this figure](https://raw.githubusercontent.com/ros-planning/navigation2/eloquent-devel/doc/architecture/navigation_overview.png)) allows us to move robot from point A to point B by creating a global plan and avoiding local obstacles.
It is integrated into e-puck example and you can run it by including `nav` parameter:

```
ros2 launch webots_ros2_epuck robot_tools_launch.py rviz:=true nav:=true mapper:=true fill_map:=false
```
or without RViz2 you can just publish a desired pose:
```
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped \"
pose:
  position:
    x: 0.3
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```

In case the navigation fails, you can restart `bt_navigator`:
```
ros2 service call /bt_navigator/change_state lifecycle_msgs/ChangeState "{transition: {id: 4}}"
ros2 service call /bt_navigator/change_state lifecycle_msgs/ChangeState "{transition: {id: 3}}"
```

![Demo](./assets/nav2.gif)

This example will work properly only for [ROS2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/) (the first ROS2 release that has a long support - 3+ years):
- Navigation2 stack is tested with version `e3469486675beb3` that includes [fix of progress checker parameters](https://answers.ros.org/question/344004/configuring-the-progress-checker-in-navigation2/) and [namespaced plugins for servers](https://github.com/ros-planning/navigation2/pull/1468).
- RViz2 for Eloquent has a bug and [cannot show a local cost map](https://github.com/ros-planning/navigation2/issues/921), therefore, `ff8fcf9a2411` (or up) version of RViz2 is desired.


### Mapping
Unfortunately, default SLAM implementation doesn't work well with e-puck.
Therefore, we created a simple mapping node that relies purely on odometry.
You can launch it as a part of the e-puck example launch file by adding `mapper` parameter:
```
ros2 launch webots_ros2_epuck robot_tools_launch.py rviz:=true mapper:=true
```
Drive the robot around (with e.g. `teleop_twist_keyboard`) to discover as much of the map as possible.
![Mapping process](./assets/mapping.gif) 

Once you are sattisfied with the result you can save the map as:
```
ros2 run nav2_map_server map_saver -f $HOME/Pictures/map
```
and load it later to use it with e.g. navigation (`t1` and `t2` represent 2 different terminals):
```
t1$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/Pictures/map.yaml -p use_sim_time:=true
t2$ ros2 service call /map_server/change_state lifecycle_msgs/ChangeState "{transition: {id: 1}}"
t2$ ros2 service call /map_server/change_state lifecycle_msgs/ChangeState "{transition: {id: 3}}"
t2$ ros2 launch webots_ros2_epuck robot_tools_launch.py rviz:=true nav:=true
```

### Differential Drive Calibration
Based on the rotation speed of each wheel and two constants, distance between the wheels and wheel radius, we can calculate the position of the robot in the local frame.
Therefore, the precision of the position estimation depends a lot on the distance between the wheels and the wheel radius.
Those constants can vary from robot to robot and here we provide a tool to help you to calibrate it (this technique is very similar to the one proposed in [_"Measurement and Correction of Systematic Odometry Errors in Mobile Robots"_](http://www-personal.umich.edu/~johannb/Papers/paper58.pdf)).

First, we want to measure wheel radius and we can achieve it by letting the robot move in a straight line:
```
ros2 run webots_ros2_epuck drive_calibrator --ros-args -p type:=linear
```
If the robot overshoots the given distance (default 0.1335m), we should increase the wheel radius, otherwise decrease it:
```
ros2 param set /epuck_driver wheel_radius 0.0215
```

Second, to calibrate the distance between the wheel we can let the robot rotate in the spot:
```
ros2 run webots_ros2_epuck drive_calibrator --ros-args -p type:=angular
```
If overshoots, the given number of rotations (default 4) then decrease the distance between the wheels, otherwise increase it.
```
ros2 param set /epuck_driver wheel_distance 0.0514
```

Third, repeat those two steps until you are satisfied with the precision.

Note that you can measure distance by a tool (e.g. ruler), it is a good initial guess, but with the technique described above, you will achieve much better results.
According to the paper, such techniques can significantly reduce systematic errors (page 4 and 23).
