import sys
sys.path.append('/home/lukic/webots2/lib/controller/python38')
from controller import Robot


class MyPlugin:
    def my_func(self):
        print('my_func called')
        robot = Robot.getInstance()
        while robot.step(32) != -1:
            pass
