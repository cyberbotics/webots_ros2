[![Build Status](https://travis-ci.com/cyberbotics/webots_ros2.svg?branch=master)](https://travis-ci.com/cyberbotics/webots_ros2) [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![Version](https://img.shields.io/github/v/tag/cyberbotics/webots_ros2?label=version)](http://wiki.ros.org/webots_ros2)
# Launch Node and Simulation

The following procedure should be used to compile the package and launch the simulation and node.

```
export WEBOTS_HOME=~/Downloads/webots
source /opt/ros/dashing/setup.bash
colcon build
source install/setup.bash
source install/local_setup.bash
ros2 launch webots_ros2_universal_robot universal_robot.launch.py  # ros2 run webots_ros2_examples example_controller
```

## Test Action Server

The action server can be tested directly using the ROS2 action CLI interface:

```
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [
      { positions: [3.02, -1.63, -1.88, 1.01, 1.51, 1.13], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 5, nanosec: 500 } },
      { positions: [-1.01, 0.38, -0.63, -0.88, 0.25, -1.63], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 5, nanosec: 500 } },
      { positions: [-1.01, 0.38, -0.63, -0.88, 0.25, 6.2], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 50, nanosec: 500 } }
    ]
  }
}"
```
