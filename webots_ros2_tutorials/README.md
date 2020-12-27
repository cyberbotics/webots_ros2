# Line Following Custom Robot
This [6th video](https://www.youtube.com/watch?v=ZTJa5f5F5fU) in the [Webots-ROS YouTube Tutorial Series](https://www.youtube.com/watch?v=jU_FD1_zAqo&list=PLt69C9MnPchkP0ZXZOqmIGRTOch8o9GiQ) implements the concept of a master and slave where the master publishes msgs on a topic and the slave subscribes to them.  Here a custom robot is made to follow a line. 3 sensors in front of the robot detects the color below it, using which we know the difference between the black and the blue color to make sure that the robot follows the black color. Using rqt_graph (visualization of nodes and topics) we can see that the master node subscribes to the sensor topics and does calculations and publishes to /cmd_vel topic which gets subscribed to by the slave node. Here the slave node acts as the bridge between Webots and ROS2 and the master node is like the brain written in ROS2.

The launch file structure in ROS2 has also been explained in the video which includes the package and core directory. This is the robot_launch file which also has the package and the executable names. 

### Build the project using:
`colcon build`
### Launch the project using: 
``ros2 launch webots_ros2_tutorial line_following.launch``

``ros2 topic list`` shows the list of all topics. 

``ros2 topic echo /topic_name`` can be used to subscribe to an topic.
![asd |20x10](images/gsod_06.png)

# Implementation of SLAM toolbox in an unknown environment
### Build the project:
``
cd ~/ros_ws/ 
colcon build
source install/setup.bash
``
The colcon build builds all the packages in the repository. 
Sourcing the package is a mandatory step so that ROS2 can register all the packages in the repository.
### Run the project:
On first terminal:
```
cd ~/ros2_ws/src/webots_ros2/webots_ros2_tutorials/config/
ros2 run slam_toolbox async_slam_toolbox_node  --ros-args --param use_sim_time:=true --params-file slam_config.yaml
```
On second terminal:
```
cd ~/ros2_ws
ros2 launch webots_ros2_tutorials slam_toolbox_launch.py
```
On third terminal:
```
rviz2
``` 
If you want the same configuration as in the video you can load it from rviz folder.

#  Implementation of AR-tag detection and getting exact pose from camera. (Integration of OpenCV)
### Build the project: 
```
cd ~/AR_ws/ 
colcon build
source install/setup.bash
```
The reason to build the packages and source as already been explained above.

### Run the project:
```
ros2 launch webots_ros2_tutorials ar_detection_launch.py
```
