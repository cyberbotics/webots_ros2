# Copyright 1996-2021 Cyberbotics Ltd.
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

from sensor_msgs.msg import JointState
from rclpy.time import Time
from webots_ros2_core.webots.controller import Node


class JointStatePublisher:
    """
    Publishes joint states.

    Discovers all joints with positional sensors and publishes corresponding ROS2 messages of type
    [`sensor_msgs/JointState`](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg).

    Args:
    ----
        robot (WebotsNode): Webots Robot node.
        jointPrefix (str): Prefix to all joint names.
        node (Node): ROS2 node.

    """

    def __init__(self, robot, joint_prefix, node, frame_id='joint_states'):
        """Initialize the position sensors and the topic."""
        self.__robot = robot
        self.__frame_id = frame_id
        self.__joint_prefix = joint_prefix
        self.__node = node
        self.__sensors = []
        self.__timestep = int(robot.getBasicTimeStep())
        self.__last_joint_states = None
        self.__previous_time = 0
        self.__previous_position = []
        self.__joint_names = []

        for i in range(robot.getNumberOfDevices()):
            device = robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.POSITION_SENSOR:
                motor = device.getMotor()
                name = motor.getName() if motor is not None else device.getName()
                self.__joint_names.append(name)
                self.__sensors.append(device)
                self.__previous_position.append(0)
                device.enable(self.__timestep)
        self.__publisher = self.__node.create_publisher(JointState, 'joint_states', 1)

    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        msg.header.stamp = Time(seconds=self.__robot.getTime()).to_msg()
        msg.header.frame_id = self.__frame_id
        msg.name = [s + self.__joint_prefix for s in self.__joint_names]
        msg.position = []
        time_difference = self.__robot.getTime() - self.__previous_time
        for i in range(len(self.__sensors)):
            value = self.__sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.__previous_position[i]) /
                                time_difference if time_difference > 0 else 0.0)
            self.__previous_position[i] = value
        msg.effort = [0.0] * 6
        self.__publisher.publish(msg)
        self.__last_joint_states = msg
        self.__previous_time = self.__robot.getTime()
