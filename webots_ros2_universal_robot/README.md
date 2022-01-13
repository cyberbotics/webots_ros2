# Universal Robot

This package provides an interface between ROS2 and the [UR3e, UR5e and UR10e simulation models](https://cyberbotics.com/doc/guide/ure) of the [Universal Robots](https://www.universal-robots.com) running in Webots.
It includes several simulations of these robots.

Documentation is available [here](https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots).

## Creation of the UR5e URDF file

The [URDF file](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_universal_robot/resource/ur5e_with_gripper.urdf) for the UR5e robot used in this package is a mix of the UR5e robot taken from [UniversalRobots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and the 3-fingers-gripper taken from [robotiq](https://github.com/ros-industrial/robotiq). Some adaptations have been made in the files from these two repositories.

### In order to reproduce a Xacro file that can generate this URDF file do follow this steps:
- Clone the repositories of [UniversalRobots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and [robotiq](https://github.com/ros-industrial/robotiq) in your package.
- Create your own Xacro file and combine the [ur.urdf.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/urdf/ur.urdf.xacro) Xacro and the [robotiq-3f-gripper_articulated_macro.xacro](https://github.com/ros-industrial/robotiq/blob/kinetic-devel/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated_macro.xacro) Xacro.
- Add the following in your new Xacro file after `<robot xmlns:xacro="http://wiki.ros.org/xacro">`:
```
<webots>
  <plugin type="webots_ros2_control::Ros2Control" />
</webots>
```

#### Steps on the `UniversalRobots` files:
- Define in the `ur.urdf.xacro` file the `joint_limit_params`, `kinematics_params`, `physical_params` and `visual_params` default arguments with the path to their corresponding `.yaml` [files](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy/ur_description/config/ur5e).
- ? remove fake ?????? normally not necessary
- In the file [ur_macro.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/urdf/ur_macro.xacro), remove the 4 lines below `<!-- Data files required by the UR driver -->` and replace the tag `<xacro:ur_ros2_control.../>` below `<!-- ros2 control instance -->` by:
```
<xacro:ur_ros2_control
    name="WebotsControl" prefix="${prefix}"
    initial_positions="${initial_positions}" />
```
- In the file [ur.ros2_control.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/urdf/ur.ros2_control.xacro) remove `script_filename output_recipe_filename input_recipe_filename tf_prefix hash_kinematics robot_ip` from the `<xacro:macro...>` tag, remove the `<joint name="speed_scaling">`, the `<joint name="gpio">`, the `<joint name="resend_robot_program">` and the `<joint name="system_interface">` tags, remove all the `<command_interface name="velocity">...</command_interface>` tags and all the `<param...>...</param>` tags from the `<command_interface name="position">...</command_interface>` tags, replace the `<hardware></hardware>` tag by:
```
<hardware>
    <plugin>webots_ros2_control::Ros2ControlSystem</plugin>
</hardware>
```
, and finally, in order to also control the gripper, add after the `<sensor name="tcp_fts_sensor">...</sensor>` these tags:
```
<!-- ROBOTIQ 3F Gripper -->
<joint name="palm_finger_1_joint">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_1_joint_1">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_1_joint_2">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_1_joint_3">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="palm_finger_2_joint">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_2_joint_1">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_2_joint_2">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_2_joint_3">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_middle_joint_1">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_middle_joint_2">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
<joint name="finger_middle_joint_3">
    <state_interface name="position"/>
    <command_interface name="position"/>
</joint>
```

#### Steps on the `robotiq` files:
- In the `robotiq-3f-gripper_articulated_macro.xacro` remove the `<link name="${prefix}tool0"/>` tag and replace the `<joint name="${prefix}palm_tool0" type="fixed">...</joint>` tags by:
```
<joint name="${prefix}tool0_palm" type="fixed">
    <parent link="${prefix}tool0"/>
    <child link="${prefix}palm"/>
    <origin xyz="0 0.045 0" rpy="1.5708 0 0"/>
</joint>
```

- And finally update accordingly to your package all the paths in the modified files, in the `visual_parameters.yaml` file and in the [robotiq-3f-gripper_finger_articulated_macro.xacro](https://github.com/ros-industrial/robotiq/blob/kinetic-devel/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_finger_articulated_macro.xacro) file.