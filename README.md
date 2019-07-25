# Launch Node and Simulation

The following procedure should be used to compile the package and launch the simulation and node.

```
export WEBOTS_HOME=~/Downloads/webots
source /opt/ros/dashing/setup.bash
colcon build --symlink-install
source install/setup.bash
source install/local_setup.bash
ros2 launch webots_ros2 launcher.launch.py  # ros2 run webots_ros2 example_controller
```

## Test Action Server

The action server can be tested directly using the ROS2 action CLI interface:

```
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [ { positions: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 3, nanosec: 500 } } ]
  }
}"
```
