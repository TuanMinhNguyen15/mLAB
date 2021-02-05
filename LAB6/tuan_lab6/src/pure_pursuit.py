#!/usr/bin/env python
import rospy
import math
from numpy import genfromtxt
import numpy as np
import pandas as pd
import tf
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

L = 1

# TODO: import ROS msg types and libraries
class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.drive_pub = rospy.Publisher('/nav',AckermannDriveStamped,queue_size=5)
        csv_file = '/home/tuan/catkin_ws/src/tuan_lab6/src/waypoints.csv'
        self.waypoint_array = genfromtxt(csv_file,delimiter=',')
        

    def odom_callback(self, odom_msg):
        global L
        x_car = odom_msg.pose.pose.position.x
        y_car = odom_msg.pose.pose.position.y
        pose_car = np.array([[x_car,y_car],]*len(self.waypoint_array))  # array

        quaternion = np.array([odom_msg.pose.pose.orientation.x, 
                           odom_msg.pose.pose.orientation.y, 
                           odom_msg.pose.pose.orientation.z, 
                           odom_msg.pose.pose.orientation.w])

        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta_car = euler[2]   # angle orientation of car or rotation angle of basis vectors

        M = np.array([[math.cos(theta_car), -math.sin(theta_car)],[math.sin(theta_car), math.cos(theta_car)]])
        M_inv = np.linalg.inv(M)    #rotation matrix
        
        waypoint_array_car = self.waypoint_array - pose_car
        waypoint_array_car = (M_inv@waypoint_array_car.T).T
        waypoint_array_car = waypoint_array_car[waypoint_array_car[:,0]>0]    # only choose points that are in front of the car
        """
        distance_list = []
        for waypoint_car in waypoint_array_car:
            distance_list.append(np.linalg.norm(waypoint_car))

        distance_list = np.array(distance_list)
        """
        distance_list = np.sum(np.abs(waypoint_array_car)**2,axis=-1)**(1./2)
        distance_list_difference = abs(distance_list - L)

        waypoint_chosen = waypoint_array_car[np.argmin(distance_list_difference)]    # chosen way point
        
        curv = 2*waypoint_chosen[1]/(L**2)
        
        steering_angle = 1*curv
        steering_angle = max(-math.pi/4,min(steering_angle,math.pi/4))
       # print(steering_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = 4
        self.drive_pub.publish(drive_msg)
        


def main():
    
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()

if __name__ == '__main__':
    main()
