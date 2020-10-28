import rclpy
import sys
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rclpy.node import Node
import time

class LineFollower(Node):
    def __init__ (self):
        super().__init__('linefollower_cmdvel')
        # Subscribe IR sensors
        self.subs_right_ir = self.create_subscription(Float64,'right_IR',self.rightIR_cb,1)
        self.subs_left_ir = self.create_subscription(Float64,'left_IR',self.leftIR_cb,1)
        self.subs_mid_ir = self.create_subscription(Float64,'mid_IR',self.midIR_cb,1)
        # Publish cmd vel
        self.pubs_cmdvel= self.create_publisher(Twist, 'cmd_vel',1)
        
        # vehicle parameters
        self.speed=0.2
        self.angle_correction=0.01

        # Initialize parameters
        self.GS_RIGHT,self.GS_MID,self.GS_LEFT=0,0,0
        self.DeltaS=0
        self.cmd=Twist()
        self.stop=False
        self.count=0
        self.count_threshold=10

    def LineFollowingModule(self):
        # Constant velocity
        self.cmd.linear.x=self.speed
        
        # Correction parameters
        self.DeltaS = self.GS_RIGHT - self.GS_LEFT
        self.cmd.angular.z=self.angle_correction*self.DeltaS

        # Logic for stop if black line not seen .
        if self.GS_RIGHT>500 and self.GS_LEFT>500 and self.GS_MID>500 :
            self.count+=1
        else:
            self.count=0

        if self.count>self.count_threshold:
            self.stop=True

        if self.stop:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z= 0.0
        
        # Publish cmd vel
        self.pubs_cmdvel.publish(self.cmd)
        self.stop=False

    # Call backs to update sensor reading variables
    def rightIR_cb(self,msg):
        self.GS_RIGHT=msg.data
        self.LineFollowingModule()

    def leftIR_cb(self,msg):
        self.GS_LEFT=msg.data
    
    def midIR_cb(self,msg):
        self.GS_MID=msg.data

def main(args=None):

    rclpy.init(args=args)

    ls=LineFollower()
    rclpy.spin(ls)
    
    ls.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()