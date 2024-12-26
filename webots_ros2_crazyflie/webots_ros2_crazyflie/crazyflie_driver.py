#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2023 Bitcraze

# @file crazyflie_driver.py
# Controls the crazyflie motors in Webots using ROS2.

"""crazyflie_driver"""


import math
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

from math import cos, sin


FLYING_ATTITUDE = 1


class CrazyflieDriver:

    def init(self, webots_node, properties):

        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        # Initialize Sensors
        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.range_front = self.robot.getDevice("range_front")
        self.range_front.enable(self.timestep)
        self.range_left = self.robot.getDevice("range_left")
        self.range_left.enable(self.timestep)
        self.range_back = self.robot.getDevice("range_back")
        self.range_back.enable(self.timestep)
        self.range_right = self.robot.getDevice("range_right")
        self.range_right.enable(self.timestep)

        # Initialize variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_time = self.robot.getTime()

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.robot.getTime()
        self.sensor_read_last_time = self.robot.getTime()

        self.vel_cmd_twist = Twist()
        self.height_desired = FLYING_ATTITUDE

        # Intialize ROS
        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_driver')
        self.node.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.laser_publisher = self.node.create_publisher(
            LaserScan, '/scan', 1)
        self.static_broadcaster = TransformBroadcaster(self.node)
        self.first_time = True

    def cmd_vel_callback(self, twist):
        self.vel_cmd_twist = twist

    def step(self):

        rclpy.spin_once(self.node, timeout_sec=0)
        dt = self.robot.getTime() - self.past_time

        if self.first_time:
            self.past_x_global = self.gps.getValues()[0]
            self.past_y_global = self.gps.getValues()[1]
            self.past_time = self.robot.getTime()
            self.first_time = False
        else:
            dt = self.robot.getTime() - self.past_time

        # Get sensor data
        roll = self.imu.getRollPitchYaw()[0]
        pitch = -self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        yaw_rate = self.gyro.getValues()[2]
        altitude = self.gps.getValues()[2]
        x_global = self.gps.getValues()[0]
        v_x_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1]
        v_y_global = (y_global - self.past_y_global)/dt
        z_global = self.gps.getValues()[2]

        # Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = - v_x_global * sinyaw + v_y_global * cosyaw

        # Initialize values
        forward_desired = self.vel_cmd_twist.linear.x
        sideways_desired = self.vel_cmd_twist.linear.y
        yaw_desired = self.vel_cmd_twist.angular.z
        height_diff_desired = self.vel_cmd_twist.linear.z

        self.height_desired += height_diff_desired * dt

        # Example how to get sensor data
        ranges = [self.range_back.getValue()/1000.0, self.range_left.getValue()/1000.0,
                  self.range_front.getValue()/1000.0, self.range_right.getValue()/1000.0]

        # Publish laser scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.node.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'crazyflie'
        scan_msg.angle_min = -0.5 * 2 * math.pi
        scan_msg.angle_max = 0.25 * 2 * math.pi
        scan_msg.angle_increment = 1.0 * math.pi/2
        scan_msg.range_min = 0.1
        scan_msg.range_max = 2.0
        scan_msg.ranges = ranges
        self.laser_publisher.publish(scan_msg)

        # Publish static transform
        laser_transform = TransformStamped()
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.node.get_clock().now().to_msg()
        laser_transform.header.frame_id = 'odom'
        laser_transform.child_frame_id = 'crazyflie'
        laser_transform.transform.translation.x = x_global
        laser_transform.transform.translation.y = y_global
        laser_transform.transform.translation.z = z_global
        laser_transform.transform.rotation.x = q[0]
        laser_transform.transform.rotation.y = q[1]
        laser_transform.transform.rotation.z = q[2]
        laser_transform.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(laser_transform)

        # PID velocity controller with fixed height
        motor_power = self.PID_CF.pid(dt, forward_desired, sideways_desired,
                                      yaw_desired, self.height_desired,
                                      roll, pitch, yaw_rate,
                                      altitude, v_x, v_y)

        self.m1_motor.setVelocity(-motor_power[0])
        self.m2_motor.setVelocity(motor_power[1])
        self.m3_motor.setVelocity(-motor_power[2])
        self.m4_motor.setVelocity(motor_power[3])

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global


class pid_velocity_fixed_height_controller():
    def __init__(self):
        self.past_vx_error = 0.0
        self.past_vy_error = 0.0
        self.past_alt_error = 0.0
        self.past_pitch_error = 0.0
        self.past_roll_error = 0.0
        self.altitude_integrator = 0.0
        self.last_time = 0.0

    def pid(self, dt, desired_vx, desired_vy, desired_yaw_rate, desired_altitude, actual_roll, actual_pitch, actual_yaw_rate,
            actual_altitude, actual_vx, actual_vy):
        # Velocity PID control (converted from Crazyflie c code)
        gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
                 "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 5, "kd_z": 5}

        # Velocity PID control
        vx_error = desired_vx - actual_vx
        vx_deriv = (vx_error - self.past_vx_error) / dt
        vy_error = desired_vy - actual_vy
        vy_deriv = (vy_error - self.past_vy_error) / dt
        desired_pitch = gains["kp_vel_xy"] * \
            np.clip(vx_error, -1, 1) + gains["kd_vel_xy"] * vx_deriv
        desired_roll = -gains["kp_vel_xy"] * \
            np.clip(vy_error, -1, 1) - gains["kd_vel_xy"] * vy_deriv
        self.past_vx_error = vx_error
        self.past_vy_error = vy_error

        # Altitude PID control
        alt_error = desired_altitude - actual_altitude
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt
        alt_command = gains["kp_z"] * alt_error + gains["kd_z"] * alt_deriv + \
            gains["ki_z"] * np.clip(self.altitude_integrator, -2, 2) + 48
        self.past_alt_error = alt_error

        # Attitude PID control
        pitch_error = desired_pitch - actual_pitch
        pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        roll_error = desired_roll - actual_roll
        roll_deriv = (roll_error - self.past_roll_error) / dt
        yaw_rate_error = desired_yaw_rate - actual_yaw_rate
        roll_command = gains["kp_att_rp"] * \
            np.clip(roll_error, -1, 1) + gains["kd_att_rp"] * roll_deriv
        pitch_command = -gains["kp_att_rp"] * \
            np.clip(pitch_error, -1, 1) - gains["kd_att_rp"] * pitch_deriv
        yaw_command = gains["kp_att_y"] * np.clip(yaw_rate_error, -1, 1)
        self.past_pitch_error = pitch_error
        self.past_roll_error = roll_error

        # Motor mixing
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]
