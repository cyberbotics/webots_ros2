# webots_ros2_core

This package contains essential building blocks for running Webots simulation, such as Webots launcher, ROS2 wrappers for Webots devices and other relevant utils.

## Reference Manual

###  webots_launcher

The `webots_launcher` is used to start Webots from your launch file, it has the following arguments:
- `--world`: defines the path to the simulation world file to load.
- `--mode`: defines the simulation mode (pause, realtime, run or fast) with which Webots should be started (realtime is set by default). 
- `--no-gui`: if set, Webots starts with a minimal graphical user interface, this is useful to use on a server for example.

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
In `webots_ros2_core` package, we provide launcher that should automatically create ROS2 services and topics based on Webots' robot description (popularly called [ROSification](https://roscon.ros.org/2013/wp-content/uploads/2013/06/ROSCon2013_rosify_robot.pdf)).
It is enough to provide path to Webots world file with the robot inside, for example: 
```
ros2 launch webots_ros2_core robot_launch.py world:=$(ros2 pkg prefix webots_ros2_universal_robot --share)/worlds/universal_robot_rviz.wbt
```
This command will run Webots with [UR5](https://www.universal-robots.com/products/ur5-robot/) and publish joint state positions, transformations and robot description.

Similarly, you can try with TIAGo++:
```
ros2 launch webots_ros2_core robot_launch.py world:=$(ros2 pkg prefix webots_ros2_tiago --share)/worlds/tiago++_example.wbt
```

### Custom Launcher File and Driver
In case a Webots device is not covered by the universal launcher or you prefer to create ROS interface differently you can build your ROS2 driver from scratch.
First, make sure you have created a new ROS2 package and call it `my_webots_driver` (you check ROS' tutorial given [here](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)).
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
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_core.utils import ControllerLauncher


def generate_launch_description():
    synchronization = LaunchConfiguration('synchronization', default=False)

    # Webots
    webots = Node(
        package='webots_ros2_core',
        node_executable='webots_launcher',
        arguments=arguments = [
            '--mode=realtime',
            '--world=' + path_to_webots_world_file
        ]
    )

    # Driver node
    controller = ControllerLauncher(
        package='webots_ros2_epuck',
        node_executable='driver',
        parameters=[{'synchronization': synchronization}]
    )

    return LaunchDescription([
        webots,
        controller,
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
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


### Examples
This Github repository contains a few good examples that you can use as the starting point:
- `webots_ros2_example` includes a very simple controller for Thymio (differential driver robot).
- `webots_ros2_tiago` is another differential drive robot, but here `WebotsDifferentialDriveNode` class from `webots_ros2_core` is utilized to simplify differential drive implementation.
- `webots_ros2_epuck` is again differential drive robot that creates ROS services and topics for almost all available sensors and actuators.
This example also contains a [list of instructions](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/EPUCK_ROS2.md) that explains how the simulation can be used in combination with different ROS2 packages like RViz and Navigation2.
Also, you will this example useful if you are planning later to go to the real robot as we support [ROS2 driver for the real robot](https://github.com/cyberbotics/epuck_ros2) as well.