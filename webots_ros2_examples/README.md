# webots_ros2_examples

In this package you can find a collection of simple examples.

## Thymio II

This package provides a very simple ROS2 node, launch file and simulation for Thymio II robot. It is a good starting point to create your own simulation and ROS2 interface with your own simulated robot.

### Launching the Simulation
The package includes a launch file that will start Webots and the node that interfaces Webots and ROS2. This launch file can be called like this:

```bash
ros2 launch webots_ros2_examples example.launch.py
```
![Thymio II in Webots](assets/ros_example.png)
Webots will start with the Thymio II differential wheel robot and the ''webots_ros2_examples'' node will start.
This node acts as a Webots robot controller and publishes the value of the front distance-sensors of the robot on the `/sensor` topic and provides the `/motor` topic which can be used to change the speed of each wheel of the robot.

To quit the simulation and stop the launch file, you simply need to close Webots.

### Interact with the Robot

#### Move the Robot
The `/motor` service can be tested directly using the [ROS2 service CLI interface](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools) to move the robot:

```bash
ros2 service call /motor webots_ros2_msgs/SetDifferentialWheelSpeed "{ left_speed: 1.0, right_speed: 0.5 }"
```

#### Display the Sensor Value
The output of the sensor (`/sensor` topic) can be displayed directly using the [ROS2 topic CLI interface](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools)

```bash
ros2 topic echo /sensor
```

#### Troubleshooting

If you see import failures or some indications saying that `WEBOTS_HOME` is incorrectly set, make sure your environment variables are [configured for extern controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?version=master#environment-variables).


## TurtleBot3 Burger

![TurtleBot3 Burger in Webots](./assets/turtlebot3_burger.png)

To run TurtleBot3 Burger simulation you can use the universal launcher:
```bash
ros2 launch webots_ros2_core robot_launch.py \
    executable:=webots_differential_drive_node \
    node_parameters:=$(ros2 pkg prefix webots_ros2_examples --share)/resource/turtlebot3_burger.yaml \
    world:=$(ros2 pkg prefix webots_ros2_examples --share)/worlds/turtlebot3_burger_example.wbt
```

If you have [`turtlebot3`](https://github.com/ROBOTIS-GIT/turtlebot3) package installed you can start `cartographer` as:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

Also, you can start `navigation2` using a custom map:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=true \
    map:=$(ros2 pkg prefix webots_ros2_examples --share)/resource/turtlebot3_burger_example_map.yaml
```

Make sure you set initial pose by clicking `2D Pose Estimate` button in RViz or by executing the following command:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
    "header": { "frame_id": "map" },
    "pose": {
        "pose": {
            "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
            "orientation": { "x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0 }
        }
    }
}'
```

> On the [official TurtleBot3 website](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#turtlebot3) you can find more information about [navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_navigation2/) and [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_slam/) configuration.

This will expose necessary topics:
```bash
$ ros2 topic list -t
/scan [sensor_msgs/msg/LaserScan]
/cmd_vel [geometry_msgs/msg/Twist]
/imu [sensor_msgs/msg/Imu]
/joint_states [sensor_msgs/msg/JointState]
/odom [nav_msgs/msg/Odometry]
/robot_description [std_msgs/msg/String]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
```

## Khepera IV

![Khepera IV in Webots](./assets/khepera4.png)

Similarly to TurtleBot3 Burger you can start Khepera IV robot as well:
```bash
ros2 launch webots_ros2_core robot_launch.py \
    executable:=webots_differential_drive_node \
    node_parameters:=$(ros2 pkg prefix webots_ros2_examples --share)/resource/khepera4.yaml \
    world:=$(ros2 pkg prefix webots_ros2_examples --share)/worlds/khepera4_example.wbt
```

For this robot you should expect the following topics:
```bash
$ ros2 topic list -t
/camera/camera_info [sensor_msgs/msg/CameraInfo]
/camera/image_raw [sensor_msgs/msg/Image]
/cmd_vel [geometry_msgs/msg/Twist]
/front_infrared_sensor [sensor_msgs/msg/Range]
/front_left_infrared_sensor [sensor_msgs/msg/Range]
/front_left_led [std_msgs/msg/Int32]
/front_left_ultrasonic_sensor [sensor_msgs/msg/Range]
/front_right_infrared_sensor [sensor_msgs/msg/Range]
/front_right_led [std_msgs/msg/Int32]
/front_right_ultrasonic_sensor [sensor_msgs/msg/Range]
/front_ultrasonic_sensor [sensor_msgs/msg/Range]
/ground_front_left_infrared_sensor [sensor_msgs/msg/Range]
/ground_front_right_infrared_sensor [sensor_msgs/msg/Range]
/ground_left_infrared_sensor [sensor_msgs/msg/Range]
/ground_right_infrared_sensor [sensor_msgs/msg/Range]
/imu [sensor_msgs/msg/Imu]
/joint_states [sensor_msgs/msg/JointState]
/left_infrared_sensor [sensor_msgs/msg/Range]
/left_ultrasonic_sensor [sensor_msgs/msg/Range]
/odom [nav_msgs/msg/Odometry]
/rear_infrared_sensor [sensor_msgs/msg/Range]
/rear_led [std_msgs/msg/Int32]
/rear_left_infrared_sensor [sensor_msgs/msg/Range]
/rear_right_infrared_sensor [sensor_msgs/msg/Range]
/right_infrared_sensor [sensor_msgs/msg/Range]
/right_ultrasonic_sensor [sensor_msgs/msg/Range]
/robot_description [std_msgs/msg/String]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
```
