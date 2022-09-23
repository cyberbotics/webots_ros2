# Universal Robot

This package provides an interface between ROS2 and the [UR3e, UR5e and UR10e simulation models](https://cyberbotics.com/doc/guide/ure) of the [Universal Robots](https://www.universal-robots.com) running in Webots.
It includes several simulations of these robots.

Documentation is available [here](https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots).

## Creation of the UR5e URDF file

The [URDF file](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_universal_robot/resource/ur5e_with_gripper.urdf) for the UR5e robot used in this package is a mix of the UR5e robot taken from [UniversalRobots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and the 3-fingers-gripper taken from [robotiq](https://github.com/ros-industrial/robotiq).
Some adaptations have been made in the files from the two repositories.

### In order to reproduce a Xacro file that can generate this URDF file follow these steps:
- Clone the repositories of [UniversalRobots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and [robotiq](https://github.com/ros-industrial/robotiq) in your package.
- Create your own Xacro file to combine the robot and the gripper and add the `<webots>` plugin tag:
```
<?xml version="1.0"?>
<robot name="UR5e" xmlns:xacro="http://wiki.ros.org/xacro">
    <!--
        Custom mix of ur5e robot and robotiq 3f gripper for Webots.
        Based on https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
        and https://github.com/ros-industrial/robotiq.
    -->

    <webots>
        <plugin type="webots_ros2_control::Ros2Control" />
    </webots>

    <!-- ur5e robot -->
    <xacro:include filename="$(find webots_ros2_universal_robot)/resource/Universal_Robots_ROS2_Driver/ur_description/urdf/ur.urdf.xacro" />

    <!-- robotiq 3f gripper -->
    <xacro:include filename="$(find webots_ros2_universal_robot)/resource/robotiq/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated_macro.xacro" />
    <xacro:robotiq-3f-gripper_articulated prefix=""/>
</robot>
```

### Steps on the `UniversalRobots` files:

#### The [ur.urdf.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/urdf/ur.urdf.xacro) file:
- Define the `joint_limit_params`, `kinematics_params`, `physical_params` and `visual_params` default arguments with the path to their corresponding `.yaml` [files](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy/ur_description/config/ur5e).
- Update all the paths accordingly to your package.

#### The [visual_parameters.yaml](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/config/ur5e/visual_parameters.yaml) file:
- Update all the paths accordingly to your package.

#### The [ur_macro.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/urdf/ur_macro.xacro) file:
- Remove the 4 lines below `<!-- Data files required by the UR driver -->`.
- Replace the tag `<xacro:ur_ros2_control.../>` below `<!-- ros2 control instance -->` by:
```
<xacro:ur_ros2_control
    name="WebotsControl" prefix="${prefix}"
    initial_positions="${initial_positions}" />
```
- Update all the paths accordingly to your package.

#### The [ur.ros2_control.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_description/urdf/ur.ros2_control.xacro) file:
- Remove `script_filename output_recipe_filename input_recipe_filename tf_prefix hash_kinematics robot_ip` from the `<xacro:macro...>` tag.
- Remove the `<joint name="speed_scaling">`, the `<joint name="gpio">`, the `<joint name="resend_robot_program">` and the `<joint name="system_interface">` tags.
- Remove all the `<command_interface name="velocity">...</command_interface>` tags and all the `<param...>...</param>` tags from the `<command_interface name="position">...</command_interface>` tags
- Replace the `<hardware></hardware>` tag by:
```
<hardware>
    <plugin>webots_ros2_control::Ros2ControlSystem</plugin>
</hardware>
```
- In order to also control the gripper, add after the `<sensor name="tcp_fts_sensor">...</sensor>` tag these tags:
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


### Steps on the `robotiq` files:

#### The [robotiq-3f-gripper_articulated_macro.xacro](https://github.com/ros-industrial/robotiq/blob/kinetic-devel/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated_macro.xacro) file:
- Remove the `<link name="${prefix}tool0"/>` tag and replace the `<joint name="${prefix}palm_tool0" type="fixed">...</joint>` tags by:
```
<joint name="${prefix}tool0_palm" type="fixed">
    <parent link="${prefix}tool0"/>
    <child link="${prefix}palm"/>
    <origin xyz="0 0 0.045" rpy="1.5708 0 0"/>
</joint>
```
- Update all the paths accordingly to your package.

#### The [robotiq-3f-gripper_finger_articulated_macro.xacro](https://github.com/ros-industrial/robotiq/blob/kinetic-devel/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_finger_articulated_macro.xacro) file:
- Update all the paths accordingly to your package.