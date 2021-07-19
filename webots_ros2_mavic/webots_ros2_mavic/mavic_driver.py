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
K_VERTICAL_P = 3.0          # P constant of the vertical PID.
K_ROLL_P = 50.0             # P constant of the roll PID.
K_PITCH_P = 30.0            # P constant of the pitch PID.
K_YAW_P = 2.0
K_X_VELOCITY_P = 1
K_Y_VELOCITY_P = 1
K_X_VELOCITY_I = 0.01
K_Y_VELOCITY_I = 0.01
LIFT_HEIGHT = 1


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicDriver(WebotsNode):
    def __init__(self, args):
        super().__init__('mavic_driver', args)
        self.start_device_manager({
            'gyro': {'always_publish': True},
            'inertial unit': {'always_publish': True},
            'gps': {'always_publish': True}
        })

        # Sensors
        self.__gps = self.robot.getDevice('gps')
        self.__gps.enable(self.timestep)
        self.__gyro = self.robot.getDevice('gyro')
        self.__imu = self.robot.getDevice('inertial unit')

        # Propellers
        self.__propellers = [
            self.robot.getDevice('front right propeller'),
            self.robot.getDevice('front left propeller'),
            self.robot.getDevice('rear right propeller'),
            self.robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__vertical_ref = LIFT_HEIGHT
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0

        # ROS interface
        self.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def log(self, *args):
        self.get_logger().warn(' '.join([str(arg) for arg in args]))

    def step(self, ms):
        super().step(ms)

        roll_ref = 0
        pitch_ref = 0

        # Read sensors
        roll, pitch, _ = self.__imu.getRollPitchYaw()
        _, _, vertical = self.__gps.getValues()
        roll_acceleration, pitch_acceleraiton, twist_yaw = self.__gyro.getValues()
        velocity = self.__gps.getSpeed()
        if math.isnan(velocity):
            return

        # Allow high level control once the drone is lifted
        if vertical > 0.2:
            # Calculate velocity
            velocity_x = (pitch / (abs(roll) + abs(pitch))) * velocity
            velocity_y = - (roll / (abs(roll) + abs(pitch))) * velocity

            # High level controller (linear velocity)
            linear_y_error = self.__target_twist.linear.y - velocity_y
            linear_x_error = self.__target_twist.linear.x - velocity_x
            self.__linear_x_integral += linear_x_error
            self.__linear_y_integral += linear_y_error
            roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * self.__linear_y_integral
            pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * self.__linear_x_integral
            self.__vertical_ref = clamp(
                self.__vertical_ref + self.__target_twist.linear.z * (ms / 1000),
                max(vertical - 0.5, LIFT_HEIGHT),
                vertical + 0.5
            )
        vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical)

        # Low level controller (roll, pitch, yaw)
        yaw_ref = self.__target_twist.angular.z

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_ref
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleraiton + pitch_ref
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

        m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
        m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input

        # Apply control
        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity(m2)
        self.__propellers[2].setVelocity(m3)
        self.__propellers[3].setVelocity(-m4)


def main(args=None):
    rclpy.init(args=args)
    driver = MavicDriver(args=args)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
