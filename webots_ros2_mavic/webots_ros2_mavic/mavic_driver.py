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

import rclpy
from geometry_msgs.msg import Twist
from webots_ros2_core.webots_node import WebotsNode


K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
K_VERTICAL_P = 5.0          # P constant of the vertical PID.
K_ROLL_P = 30.0             # P constant of the roll PID.
K_PITCH_P = 15.0            # P constant of the pitch PID.
K_YAW_P = 2.0


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

        # ROS interface
        self.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def log(self, *args):
        self.get_logger().warn(' '.join([str(arg) for arg in args]))

    def step(self, ms):
        super().step(ms)

        roll, pitch, _ = self.__imu.getRollPitchYaw()
        _, _, vertical = self.__gps.getValues()
        x, y, twist_yaw = self.__gyro.getValues()

        # High level controller (linear and angular velocity)
        roll_ref = self.__target_twist.linear.y
        pitch_ref = self.__target_twist.linear.x

        # Low level controller (roll, pitch, yaw)
        yaw_ref = self.__target_twist.angular.z
        vertical_ref = 1

        roll_input = - K_ROLL_P * (roll_ref - roll)
        pitch_input = - K_PITCH_P * (pitch_ref - pitch)
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)
        vertical_input = K_VERTICAL_P * (vertical_ref - vertical)

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
