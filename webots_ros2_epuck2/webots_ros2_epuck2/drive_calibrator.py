import rclpy
import math
from math import pi
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import Twist


def quaternion_to_euler(q):
    # Reference: https://computergraphics.stackexchange.com/a/8229
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


class EPuckDriveCalibrator(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

        self.type = self.declare_parameter('type', 'rotation')

        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        if self.type.value == 'rotation':
            print('Rotation calibration in progress...')
            msg = Twist()
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 1.0
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            self.pub.publish(msg)

    def send_stop(self):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def odometry_callback(self, msg: Odometry):
        if self.type.value == 'rotation':
            yaw, _, _ = quaternion_to_euler(msg.pose.pose.orientation)
            print('Yaw', yaw)
            if abs(yaw - pi) < 0.1:
                self.send_stop()

def main(args=None):
    rclpy.init(args=args)
    epuck2_controller = EPuckDriveCalibrator(
        'epuck_drive_calibrator', args=args)
    rclpy.spin(epuck2_controller)
    epuck2_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
