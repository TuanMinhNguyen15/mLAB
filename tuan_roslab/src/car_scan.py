#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tuan_roslab.msg import scan_range

range_max = 0.0
range_min = 0.0

def scan_subscriber():
    rospy.Subscriber('/scan',LaserScan,scan_callback)
    
    
    

def scan_callback(data):
    global range_max , range_min
    range_array = np.array(data.ranges)
    range_max = np.amax(range_array)
    range_min = np.amin(range_array)
 
    
def scan_publisher():
    pub_range_max = rospy.Publisher('/farthest_point',Float64,queue_size=10)
    pub_range_min = rospy.Publisher('/closest_point',Float64,queue_size=10)
    pub_range = rospy.Publisher('/scan_range',scan_range,queue_size=10)

    LIDAR = scan_range()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub_range_max.publish(range_max)
        pub_range_min.publish(range_min)

        LIDAR.range_max = range_max
        LIDAR.range_min = range_min
        pub_range.publish(LIDAR)
        rate.sleep()
    

    

if __name__ == '__main__':
    try:
        rospy.init_node("car_scan",anonymous=True)
        scan_subscriber()
        scan_publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    