## Example usage:

### urdf import
```
ros2 run webots_ros2_importer urdf2proto --input=/home/user/ros2_workspace/install/turtlebot3_description/share/turtlebot3_description/urdf/turtlebot3_burger.urdf --disable-mesh-optimization --output=Turtlebot3Burger.proto
```

### xacro import
```
ros2 run webots_ros2_importer xacro2proto /home/user/ros2_workspace/install/turtlebot3_description/share/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro --disable-mesh-optimization --output=Turtlebot3Burger.proto
```
