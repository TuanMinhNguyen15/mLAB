#!/usr/bin/env python
import rospy
import numpy as np
import math

# TODO: import ROS msg types and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        rospy.Subscriber('/odom',Odometry,self.odom_callback)
        rospy.Subscriber('/scan',LaserScan,self.scan_callback)
        self.brake_pub = rospy.Publisher('/brake',AckermannDriveStamped,queue_size=10)
        self.brake_bool_pub = rospy.Publisher('/brake_bool',Bool,queue_size=10)
        self.brake_msg = AckermannDriveStamped()
        self.bool_msg = Bool()



    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
       
       
        

        

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
      
        self.ranges = scan_msg.ranges

        self.T_list = []
        for i in range(0,len(self.ranges)):
            r = self.ranges[i]
            self.angle = scan_msg.angle_min + i*scan_msg.angle_increment
            self.r_dot = -self.speed*math.cos(self.angle)
            self.T = r/(max(-self.r_dot,0.00001))
            
            self.T_list.append(self.T)

        self.TTC = min(self.T_list)
        
       
        # TODO: publish brake message and publish controller bool
        if self.TTC < 0.5:   #0.42
            self.brake_msg.drive.speed = 0
            self.bool_msg.data = True
            self.brake_pub.publish(self.brake_msg)
            self.brake_bool_pub.publish(self.bool_msg)
            print("ABE ENGAGED!!!!!")

        

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()