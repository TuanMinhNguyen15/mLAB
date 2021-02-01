#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0.8#TODO
kd = 0#TODO
ki = 0#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback)#TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=5)#TODO: Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        self.angle_inertial = angle - (math.pi)/2
        self.ranges = np.array(data.ranges)
        self.angle_index = (self.angle_inertial - data.angle_min)/data.angle_increment
        self.angle_index = round(self.angle_index)

        return self.ranges[self.angle_index]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd

        self.error_derivative = error - prev_error
        integral = error + integral
        self.error_integration = integral
        prev_error = error

        angle = -kp*error - kd*self.error_derivative + ki*self.error_integration
        angle = max(-math.pi/4,min(angle,math.pi/4))

        if angle < math.pi/18:
            velocity = 1.5
        elif angle < math.pi/9:
            velocity = 1
        else:
            velocity = 0.5
        #print(angle*180/3.14)

        #TODO: Use kp, ki & kd to implement a PID controller for 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        self.theta = math.pi/4
        self.L = 0.6
        self.b = self.getRange(data,(math.pi - 0))  # 30 degrees from negative x-axis
        self.a = self.getRange(data,(math.pi - self.theta))
        self.alpha = math.atan((self.a*math.cos(self.theta)-self.b)/(self.a*math.sin(self.theta)))
        self.D = self.b*math.cos(self.alpha) + self.L*math.sin(self.alpha)

        return leftDist - self.D 

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data,1.2) #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)