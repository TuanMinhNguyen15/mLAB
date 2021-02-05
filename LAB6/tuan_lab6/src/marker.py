#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import visualization_msgs
import numpy as np
from numpy import genfromtxt



def talker():
    count = 0
    MARKERS_MAX = 100
    csv_file = '/home/tuan/catkin_ws/src/tuan_lab6/src/waypoints.csv'
    waypoint_array = genfromtxt(csv_file,delimiter=',')

    pub = rospy.Publisher('/visualization_marker_array',MarkerArray,queue_size=3)
   
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    iden = 0
    while not rospy.is_shutdown():
        marker_array = MarkerArray() 
        marker = Marker()
        
        
     

        marker.header.frame_id = "base_link"
        marker.id = iden
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = waypoint_array[iden][0]
        marker.pose.position.y = waypoint_array[iden][1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_array.markers.append(marker)
        
        
       
           
        
        pub.publish(marker_array)

        marker_array.markers.pop(0)
        iden = iden + 1
       # marker_array.markers = []
        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass