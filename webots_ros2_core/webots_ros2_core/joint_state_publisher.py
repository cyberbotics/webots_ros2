# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Joint state publisher."""

import sys

from webots_ros2_core.utils import append_webots_python_lib_to_path

from sensor_msgs.msg import JointState
from rclpy.time import Time

try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e


class JointStatePublisher():
    """
    Publishes joint states.

    Discovers all joints with positional sensors and publishes corresponding ROS2 messages of type
    [`sensor_msgs/JointState`](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg).

    Args:
        robot (WebotsNode): Webots Robot node.
        jointPrefix (str): Prefix to all joint names.
        node (Node): ROS2 node.
    """

    def __init__(self, robot, jointPrefix, node):
        """Initialize the position sensors and the topic."""
        self.robot = robot
        self.jointPrefix = jointPrefix
        self.node = node
        self.sensors = []
        self.timestep = int(robot.getBasicTimeStep())
        self.last_joint_states = None
        self.previousTime = 0
        self.previousPosition = []
        self.jointNames = []
        for i in range(robot.getNumberOfDevices()):
            device = robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.POSITION_SENSOR:
                motor = device.getMotor()
                name = motor.getName() if motor is not None else device.getName()
                self.jointNames.append(name)
                self.sensors.append(device)
                self.previousPosition.append(0)
                device.enable(self.timestep)
        self.publisher = self.node.create_publisher(JointState, 'joint_states', 1)

    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        msg.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        msg.header.frame_id = 'From simulation state data'
        msg.name = [s + self.jointPrefix for s in self.jointNames]
        msg.position = []
        timeDifference = self.robot.getTime() - self.previousTime
        for i in range(len(self.sensors)):
            value = self.sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.previousPosition[i]) /
                                timeDifference if timeDifference > 0 else 0.0)
            self.previousPosition[i] = value
        msg.effort = [0.0] * 6
        self.publisher.publish(msg)
        self.last_joint_states = msg
        self.previousTime = self.robot.getTime()
