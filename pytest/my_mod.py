import sys
sys.path.append('/home/lukic/webots2/lib/controller/python38')
from controller import Robot


def my_func():
    robot = Robot.getInstance()
    print('my_func called')
    while robot.step(32) != -1:
        pass
