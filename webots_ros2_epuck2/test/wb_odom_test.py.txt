"""odom_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import pi, sin, cos

robot = Robot()

timestep = int(robot.getBasicTimeStep())

WHEEL_DISTANCE = 0.05685
WHEEL_RADIUS = 0.02
CAMERA_PERIOD_MS = 500
ENCODER_PERIOD_MS = timestep
DISTANCE_PERIOD_MS = timestep
ENCODER_RESOLUTION = (2 * pi) / 1000

ENCODER_PERIOD_S = ENCODER_PERIOD_MS / 1000


class Test:
    def __init__(self):
        self.robot = robot
        
        self.prev_left_wheel_ticks = 0
        self.prev_right_wheel_ticks = 0
        self.prev_position = (0, 0)
        self.prev_angle = 0
        self.left_wheel_sensor = self.robot.getPositionSensor(
            'left wheel sensor')
        self.right_wheel_sensor = self.robot.getPositionSensor(
            'right wheel sensor')
        self.left_wheel_sensor.enable(timestep)
        self.right_wheel_sensor.enable(timestep)
    
    def odometry_callback(self):
        # Calculate velocities
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()
        v_left_rad = (left_wheel_ticks - self.prev_left_wheel_ticks) / ENCODER_PERIOD_S
        v_right_rad = (right_wheel_ticks - self.prev_right_wheel_ticks) / ENCODER_PERIOD_S
        v_left = v_left_rad * WHEEL_RADIUS
        v_right = v_right_rad * WHEEL_RADIUS
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / WHEEL_DISTANCE
    
        # Calculate position & angle
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(self.prev_angle)
        k01 = v * sin(self.prev_angle)
        k02 = omega
        k10 = v * cos(self.prev_angle + ENCODER_PERIOD_S * k02 / 2)
        k11 = v * sin(self.prev_angle + ENCODER_PERIOD_S * k02 / 2)
        k12 = omega
        k20 = v * cos(self.prev_angle + ENCODER_PERIOD_S * k12 / 2)
        k21 = v * sin(self.prev_angle + ENCODER_PERIOD_S * k12 / 2)
        k22 = omega
        k30 = v * cos(self.prev_angle + ENCODER_PERIOD_S * k22 / 2)
        k31 = v * sin(self.prev_angle + ENCODER_PERIOD_S * k22 / 2)
        k32 = omega
        position = [
            self.prev_position[0] + (ENCODER_PERIOD_S / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self.prev_position[1] + (ENCODER_PERIOD_S / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self.prev_angle + (ENCODER_PERIOD_S / 6) * (k02 + 2 * (k12 + k22) + k32)
    
        # Update variables
        self.prev_position = position.copy()
        self.prev_angle = angle
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks
    
        # Pack & publish everything
        print('v = {:.2E}; omega = {:.2E}; position = ({:.2E}, {:.2E}); angle = {:.2E}; v_left = {:.2E}; v_right = {:.2E}'
            .format(v, omega, position[0], position[1], angle % (2 * pi), v_left, v_right)
        )


test = Test()

left_motor = robot.getMotor('left wheel motor')
right_motor = robot.getMotor('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(-0.9)
right_motor.setVelocity(-0.9)

steps = 700
while robot.step(timestep) != -1:
    test.odometry_callback()
    if steps < 0:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
    else:
        steps -= 1
