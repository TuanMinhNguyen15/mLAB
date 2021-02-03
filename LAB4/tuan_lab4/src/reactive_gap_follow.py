#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools
import operator

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=3) #TODO
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # step1:
        array_len = len(ranges)
        window = 5
        for i in range(0,array_len):
            sum = 0
            avg = 0
            for j in range(0,array_len):
                sum = sum + ranges[(i+j)%array_len]
            avg = sum/window
            ranges[i] = avg

    
        proc_ranges = ranges
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return None


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        rb = 0.35
        ranges = np.array(data.ranges)
        
        #proc_ranges = self.preprocess_lidar(ranges)
        scan_angle = math.pi/6
        index_scan_start = round((-scan_angle - data.angle_min)/data.angle_increment)
        index_scan_end = round((scan_angle - data.angle_min)/data.angle_increment)

        ranges_pro = ranges[index_scan_start:index_scan_end]   # new array
    
  

        #Find closest point to LiDAR

        index_min = np.argmin(ranges_pro)
        range_min = ranges_pro[index_min]
        angle_min = -scan_angle + index_min*data.angle_increment

        #Eliminate all points inside 'bubble' (set them to zero) 


        for i in range(0,len(ranges_pro)):
            angle_check = -scan_angle + i*data.angle_increment
            range_check = ranges_pro[i]

            angle_between = abs(angle_check - angle_min)
            distance_between = math.sqrt(range_min**2 + range_check**2 - 2*range_min*range_check*math.cos(angle_between))
            if distance_between <= rb:
                ranges_pro[i] = 0
        
        #Find max length gap 

        
        index_nonzero_local = []
        index_nonzero_list = []

        for k in range(0,len(ranges_pro)):
            if ranges_pro[k] != 0:
                index_nonzero_local.append(k)
                
                if k == (len(ranges_pro) - 1):
                    index_nonzero_list.append(index_nonzero_local)

            else:
                index_nonzero_list.append(index_nonzero_local)
                index_nonzero_local = []

        index_nonzero_list = [x for x in index_nonzero_list if x != []]

        index_nonzero_list_largest = max(index_nonzero_list,key=len)

        
        
        index_best = round((min(index_nonzero_list_largest)+max(index_nonzero_list_largest))/2)
        angle_best = -angle_check + index_best*data.angle_increment
        


        #Find the best point in the gap 
       # max_length_gap = ranges_pro[min(index_nonzero_list_largest):(max(index_nonzero_list_largest)+1)]
       # index_max = np.argmax(max_length_gap) + min(index_nonzero_list_largest)

        #angle_best = -angle_check + index_max*data.angle_increment


        #print(angle_best*180/3.14)
       # print(ranges_pro)

        #Publish Drive message

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle_best
        drive_msg.drive.speed = 2
        self.drive_pub.publish(drive_msg)

        

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)