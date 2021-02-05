#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

home = expanduser('~')
file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

previous_pose = np.array([1,1])

def save_waypoint(data):
    global previous_pose

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    current_pose = np.array([current_x,current_y])

    if np.linalg.norm(current_pose - previous_pose) >= 0.2:
        file.write('%f, %f\n' % (current_x,current_y))
        previous_pose = current_pose

    
    
    

    

def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    #rospy.Subscriber('pf/pose/odom', Odometry, save_waypoint)
    rospy.Subscriber('/odom', Odometry, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass