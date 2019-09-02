[![Build Status](https://travis-ci.com/cyberbotics/webots_ros2.svg?branch=master)](https://travis-ci.com/cyberbotics/webots_ros2) [![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![Version](https://img.shields.io/github/v/tag/cyberbotics/webots_ros2?label=version)](http://wiki.ros.org/webots_ros2)
# Launch Node and Simulation

The following procedure should be used to compile the package and launch the simulation and node.

```
export WEBOTS_HOME=~/Downloads/webots
source /opt/ros/dashing/setup.bash  # on Windows: call C:\dev\ros2\local_setup.bat
colcon build
source install/setup.bash  # on Windows: call install\setup.bat
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
      { positions: [-1.01, 0.38, -0.63, -0.88, 0.25, -1.63], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 6, nanosec: 500 } },
      { positions: [-1.01, 0.38, -0.63, -0.88, 0.25, 6.2], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], accelerations: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 50, nanosec: 500 } }
    ]
  }
}"
```

## Move Robot in the Example

The `/motor` service can be tested directly using the ROS2 service CLI interface to move the robot:

```
ros2 service call /motor webots_ros2_msgs/SetDifferentialWheelSpeed "{ left_speed: 1.0, right_speed: 0.5 }"
```

And the output of the sensor (`/sensor` topic) can be displayed directly using the ROS2 topic CLI interface to move the robot:
```
ros2 topic echo /sensor
```

## Other

At runtime, ROS2 will look for Webots at the following locations (in this order):
  - Folder pointed by the `ROS2_WEBOTS_HOME` environment variable.
  - Inside the `webots_ros2_desktop` package.
  - Folder pointed by the `WEBOTS_HOME` environment variable.
  - Default installation pathes (e.g. `/usr/local/webots`).
