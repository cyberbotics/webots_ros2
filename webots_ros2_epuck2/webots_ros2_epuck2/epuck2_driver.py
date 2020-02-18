# Copyright 1996-2020 Cyberbotics Ltd.
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

"""ROS2 example controller."""

from webots_ros2_core.webots_node import WebotsNode
import rclpy
from sensor_msgs.msg import Range



class EPuck2Controller(WebotsNode):
    def __init__(self, args):
        super().__init__('epuck2_controller', args)
        self.sensor_publishers = []
        self.sensors = []
        self.create_timer(0.001 * self.timestep, self.sensor_callback)

        for i in range(8):
            sensor = self.robot.getDistanceSensor('ps{}'.format(i))
            sensor.enable(self.timestep)
            sensor_publisher = self.create_publisher(Range, 'sensor/ps{}'.format(i), 10)
            self.sensors.append(sensor)
            self.sensor_publishers.append(sensor_publisher)
            
    def sensor_callback(self):
        for i in range(8):
            msg = Range()
            msg.field_of_view = 0.01
            msg.min_range = 0.1         # TODO: Take from simulation
            msg.max_range = 10.0        # TODO: Take from simulation
            msg.range = self.sensors[i].getValue()
            msg.radiation_type = Range.ULTRASOUND
            self.sensor_publishers[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    epuck2_controller = EPuck2Controller(args=args)

    rclpy.spin(epuck2_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
