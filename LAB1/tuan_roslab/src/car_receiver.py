#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def car_receiver():
    rospy.init_node('car_receiver',anonymous=True)
    rospy.Subscriber('/farthest_point',Float64,receiver_callback)
    rospy.spin()

def receiver_callback(scan):
    print(scan.data)

if __name__ == '__main__':
    car_receiver()
