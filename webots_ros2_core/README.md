# webots_ros2_core

This package contains essential building blocks for running Webots simulation, such as Webots launcher, ROS2 wrappers for Webots devices and other relevant utils.

## Reference Manual

###  webots_launcher

The `webots_launcher` is a custom ROS action used to start Webots from your launch file, it has the following parameters:
- `world`: defines the path to the simulation world file to load.
- `mode`: defines the simulation mode (pause, realtime, run or fast) with which Webots should be started (realtime is set by default).
- `gui`: if set, Webots starts with a minimal graphical user interface, this is useful to use on a server for example.

<details><summary>`webots_launcher` usage example</summary>


```Python
import launch
from launch import LaunchDescription
from webots_ros2_core.webots_launcher import WebotsLauncher


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(
        world=world,
        mode=mode,
        gui=gui
    )

    return LaunchDescription([
        webots,
        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```

</details>


### controller_launcher

The `webots_launcher` is a custom ROS node launcher used to start Webots controller. It has the same API as `launch_ros.actions.Node`, but it adds necessary libraries needed for your ROS node to work with Webots.

<details><summary>`controller_launcher` usage example</summary>

```Python
import launch
from launch import LaunchDescription
from webots_ros2_core.webots_launcher import WebotsLauncher


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(
        world=world,
        mode=mode,
        gui=gui
    )

    controller = ControllerLauncher(
        package=package,
        node_executable=executable,
        arguments=[
            '--webots-robot-name', robot_name,
            '--webots-node-name', node_name
        ],
    )

    return LaunchDescription([
        webots,
        controller,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```

</details>

### Python Modules

This package includes the following Python modules that can be used from within other nodes to easily create an interface between a simulated robot and ROS2.

#### webots_node

This module provides the `WebotsNode` class that is used as a base class for all the other nodes.
It creates the interface between Webots and ROS and publishes the clock topic.

### utils

This module provides the following utility functions:
- `get_webots_home`: returns the path to the Webots installation directory. None is returned if Webots is not found.
- `get_webots_version`: returns the version of Webots as a string.
- `append_webots_lib_to_path`: adds the Webots `lib` folder to the library path.
- `append_webots_python_lib_to_path`: adds the Webots Python API to the Python path.

### joint_state_publisher

This module provides the `JointStatePublisher` class that is used to publish joint states.

### trajectory_follower

This module provides the `TrajectoryFollower` class that is used to provide an action server to move the joints.


## Creating ROS2 Driver
ROS drivers are considered to be ROS nodes which have a tight interaction with a robot (physical or simulated).
Therefore, in the further text, we will explain how to create ROS2 node that tightly interacts with the simulated robot in Webots.


### Universal Launcher
In `webots_ros2_core` package, we provide `robot_launch.py` launcher that should automatically create ROS2 services and topics based on Webots' robot description (popularly called [ROSification](https://roscon.ros.org/2013/wp-content/uploads/2013/06/ROSCon2013_rosify_robot.pdf)).
It is enough to provide path to Webots world file with the robot inside, for example:
```
ros2 launch webots_ros2_core robot_launch.py \
    world:=$(ros2 pkg prefix webots_ros2_universal_robot --share)/worlds/universal_robot_rviz.wbt
```
This command will run Webots with [UR5](https://cyberbotics.com/doc/guide/ure) and publish joint state positions, transformations and robot description.

> Do not get confused by `$(ros2 pkg prefix webots_ros2_universal_robot --share)` as it will simply return path to share directory of `webots_ros2_universal_robot` package. Alternatively, you can specify absolute path to `universal_robot_rviz.wbt` file.

Similarly, you can try with more complex example like TIAGo++:
```
ros2 launch webots_ros2_core robot_launch.py \
    world:=$(ros2 pkg prefix webots_ros2_tiago --share)/worlds/tiago++_example.wbt
```

To run more exhaustive list of `robot_launch.py` arguments you can use `--show-args` argument:
```
ros2 launch webots_ros2_core robot_launch.py --show-arguments
```

#### Custom Configuration
The universal launcher allows fine tunning of the ROS interface through [ROS parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/).
It means that the user can disable a device, change topic name, change publishing period and similar by changing the parameters.
To check all available parameters for your robot you should start your robot first, e.g. in case of TIAGO++:
```
ros2 launch webots_ros2_core robot_launch.py \
    world:=$(ros2 pkg prefix webots_ros2_tiago --share)/worlds/tiago++_example.wbt
```
and in the other terminal run:
```
ros2 param list /webots_driver
```
to see the list of available parameters.

At this point you can also save all parameters to YAML file for later use:
```
ros2 param dump /webots_driver
```
which will save the configuration to `webots_driver.yaml` by default.
You can open this file, change the configuration and load it later using `node_parameters` argument:
```
ros2 launch webots_ros2_core robot_launch.py \
    node_parameters:=./webots_driver.yaml \
    world:=$(ros2 pkg prefix webots_ros2_tiago --share)/worlds/tiago++_example.wbt
```

All parameters are named in the following format:
- `[webots_device_name].[parameter]` for Webots devices that expose one or more topics and services (e.g. DistanceSensor).
- `[webots_device_name_1]+[webots_device_name_2]+[webots_device_name_n].[parameter]` for multiple Webots devices that are coupled to create a single topics or service (e.g. Accelerometer, Gyro and InertialUnit devices are combined to publish to `sensor_msgs/Imu` topic).
- Robot wide parameters don't have prefix (e.g. `synchronization`) and these parameters depend on Webots node implementation (e.g. `webots_differential_drive_node`).

##### Differential Drive
TIAGo++ has differential drive which has to be explicitly described.
For differential drive robots you should utilize `webots_differential_drive_node` which exposes the following parameters:
```python
wheel_distance      # Distance between the wheels (axle length) in meters
wheel_radius        # Radius of the wheels in meters
left_joint          # Name of Motor associated with the left wheel (default `left wheel motor`)
right_joint         # Name of Motor associated with the right wheel (default `right wheel motor`)
left_encoder        # Name of PositionSensor associated with the left wheel (default `left wheel sensor`)
right_encoder       # Name of PositionSensor associated with the right wheel (default `right wheel sensor`)
command_topic       # Topic name to which the node will be subscribed to receive velocity commands (of type `geometry_msgs/Twist`, default `/cmd_vel`)
odometry_topic      # Topic name to which odometry data (of type `nav_msgs/Odometry`) will be published (default `/odom`)
odometry_frame      # Name of of the odometry frame (default `odom`)
robot_base_frame    # Name of the robot base frame (default `base_link`)
```
Make sure those parameters are correctly configured otherwise the node will crash.
Minimum `wheel_distance` and `wheel_radius` are required, but you will probably need to change `left_joint`, `right_joint`, `left_encoder` and `right_encoder` to suit your robot.
In case of TIAGo++ configuration file should look like this:
```yaml
webots_driver:
  ros__parameters:
    left_encoder: wheel_left_joint_sensor
    left_joint: wheel_left_joint
    right_encoder: wheel_right_joint_sensor
    right_joint: wheel_right_joint
    wheel_distance: 0.404
    wheel_radius: 0.1955
```
Then, you can start the Webots:
```
ros2 launch webots_ros2_core robot_launch.py \
    executable:=webots_differential_drive_node \
    node_parameters:=$(ros2 pkg prefix webots_ros2_tiago --share)/resource/tiago.yaml \
    world:=$(ros2 pkg prefix webots_ros2_tiago --share)/worlds/tiago++_example.wbt
```
Now, topics `/odom` and `/cmd` should be availabe, so you can read odometry data (e.g. visualize in RViz) and control the robot (with e.g. `teleop_twist_keyboard`).

##### Robotic Arm
For robotic arm robots you should utilize `webots_robotic_arm_node` which exposes the following parameters:
```python
prefix  # prefix to be used for joints
```

This node will automatically publish the joints state and create an action server to perform trajectory following.

### Custom Launcher File and Driver
In case a Webots device is not covered by the universal launcher or you prefer to create ROS interface differently you can build your ROS2 driver from scratch.
First, make sure you have created a new ROS2 package and call it `my_webots_driver` (you can check ROS' tutorial given [here](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)).
After the package is ready, you can create a driver, e.g. `/my_webots_driver/my_webots_driver/driver.py` and populate it with the following content:

```Python
import rclpy
from webots_ros2_core.webots_node import WebotsNode


class MyWebotsDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('my_webots_driver', args=args)


def main(args=None):
    rclpy.init(args=args)
    my_webots_driver = MyWebotsDriver(args=args)
    rclpy.spin(my_webots_driver)
    my_webots_driver.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
Notice that you have to inherit `WebotsNode` which contains basic functionality which allows interaction with a robot in Webots.
Also, you need to create a launch file `/my_webots_driver/launch/robot_launch.py` with the minimal content as following:
```Python
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'my_webots_driver'),
            ('executable', 'driver'),
            ('world', path_to_webots_world_file),
        ]
    )

    return LaunchDescription([
        webots
    ])
```
The purpose of the launch file is to start Webots, your driver for Webots and to make sure everything is stopped once Webots closed.
Then, make sure the driver and the launch file are added to `setup.py`, run `colcon build` and your launch file should be ready to be executed:
```
ros2 launch my_webots_driver robot_launch.py
```
To extend the ROS interface you should go back to `/my_webots_driver/launch/robot_launch.py` and implement more features.
For example, in order to add a basic support for [DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor) `MyWebotsDriver` class can be extended as follows:
```Python
class MyWebotsDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('my_webots_driver', args=args)
        self.sensor = self.robot.getDistanceSensor('my_distance_sensor')
        self.sensor.enable(self.timestep)
        self.sensor_publisher = self.create_publisher(Range, '/my_distance_sensor', 1)
        self.create_timer(self.timestep * 1e-3, self.publish_sensor_data)

    def publish_sensor_data(self)
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'my_distance_sensor'
        msg.field_of_view = self.sensor.getAperture()
        msg.min_range = self.sensor.getMinValue()
        msg.max_range = self.sensor.getMaxValue()
        msg.range = self.sensor.getValue()
        msg.radiation_type = Range.INFRARED
        self.sensor_publisher.publish(msg)
```

This example can work in conjunction automatic robot ROSification library provided by Webots.
Therefore, you can further extend the example above with `start_device_manager(self, config)`:
```Python
class MyWebotsDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('my_webots_driver', args=args)
        self.start_device_manager({
            'my_distance_sensor': {
                'disable': True
            }
        })
        self.sensor = self.robot.getDistanceSensor('my_distance_sensor')
        self.sensor.enable(self.timestep)
        self.sensor_publisher = self.create_publisher(Range, '/my_distance_sensor', 1)
        self.create_timer(self.timestep * 1e-3, self.publish_sensor_data)

    def publish_sensor_data(self)
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'my_distance_sensor'
        msg.field_of_view = self.sensor.getAperture()
        msg.min_range = self.sensor.getMinValue()
        msg.max_range = self.sensor.getMaxValue()
        msg.range = self.sensor.getValue()
        msg.radiation_type = Range.INFRARED
        self.sensor_publisher.publish(msg)
```
and Webots will automatically create ROS interface for other devices (other than `my_distance_sensor`) avaialble in the robot.

### Examples
This Github repository contains a few good examples that you can use as the starting point:
- `webots_ros2_example` includes a very simple controller for Thymio (differential driver robot).
- `webots_ros2_tiago` is another differential drive robot simulation, but here `WebotsDifferentialDriveNode` class from `webots_ros2_core` is utilized to simplify differential drive implementation.
- `webots_ros2_epuck` is one more example with differential drive robot in which ROS services and topics are created for almost all sensors and actuators available on the robot.
This example also contains a [list of instructions](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/EPUCK_ROS2.md) that explains how the simulation can be used in combination with different ROS2 packages like RViz and Navigation2.
Also, you will find this example useful if you plan later to control the real robot as we also support [ROS2 driver for the real robot](https://github.com/cyberbotics/epuck_ros2).
