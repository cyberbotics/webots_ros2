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

"""ROS2 Mavic 2 Pro driver."""

import math
import rclpy
from geometry_msgs.msg import Twist
from webots_ros2_core.webots_node import WebotsNode


K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
K_VERTICAL_OFFSET = 0.6     # Vertical offset where the robot actually targets to stabilize itself.
K_VERTICAL_P = 3.0          # P constant of the vertical PID.
K_ROLL_P = 50.0             # P constant of the roll PID.
K_PITCH_P = 30.0            # P constant of the pitch PID.


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('mavic_driver', args)
        self.start_device_manager({
            'gyro': {'always_publish': True},
            'inertial unit': {'always_publish': True}
        })

        # Sensors
        self.__gps = self.robot.getDevice('gps')
        self.__gps.enable(self.timestep)
        self.__gyro = self.robot.getDevice('gyro')
        self.__inertial_unit = self.robot.getDevice('inertial unit')

        # Propellers
        self.__propellers = [
            self.robot.getDevice('front left propeller'),
            self.robot.getDevice('front right propeller'),
            self.robot.getDevice('rear left propeller'),
            self.robot.getDevice('rear right propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__target_altitude = 0.1
        self.__current_twist = Twist()
        self.__previous_position = None

        # ROS interface
        self.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def log(self, *args):
        self.get_logger().warn(' '.join([str(arg) for arg in args]))

    def step(self, ms):
        super().step(ms)

        if self.__previous_position is None:
            self.__previous_position = self.__gps.getValues().copy()
            return

        # Update current state
        position = self.__gps.getValues()
        self.__current_twist.linear.x = (position[0] - self.__previous_position[0]) / (ms / 1000)
        self.__current_twist.linear.y = (position[1] - self.__previous_position[1]) / (ms / 1000)
        self.__current_twist.linear.z = (position[2] - self.__previous_position[2]) / (ms / 1000)
        self.__current_twist.angular.z = self.__gyro.getValues()[2]
        self.__previous_position = position.copy()
        self.log(self.__current_twist.linear.z)

        # Control
        error_linear_x = self.__target_twist.linear.x - self.__current_twist.linear.x
        error_linear_y = self.__target_twist.linear.y - self.__current_twist.linear.y
        error_linear_z = self.__target_twist.linear.z - self.__current_twist.linear.z

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0
        self.__target_altitude += error_linear_z

        roll = self.__inertial_unit.getRollPitchYaw()[0] + math.pi / 2
        pitch = self.__inertial_unit.getRollPitchYaw()[1]
        altitude = self.__gps.getValues()[1]
        roll_acceleration = self.__gyro.getValues()[0]
        pitch_acceleration = self.__gyro.getValues()[1]

        roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = clamp(self.__target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

        self.__propellers[0].setVelocity(K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input)
        self.__propellers[1].setVelocity(- (K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input))
        self.__propellers[2].setVelocity(- (K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input))
        self.__propellers[3].setVelocity(K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input)


def main(args=None):
    rclpy.init(args=args)
    driver = MavicDriver(args=args)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
