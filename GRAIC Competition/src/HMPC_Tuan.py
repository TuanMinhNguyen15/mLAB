import rospy
import numpy as np
import argparse
import time
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo

### Tuan Imports ###
import math
from mip import *




class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.role_name = role_name
        self.controlPub = rospy.Publisher("/carla/%s/ackermann_control"%role_name, AckermannDrive, queue_size = 1)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        self.controlPub.publish(newAckermannCmd)

    #@njit(fastmath=False, cache=True)
    def trajectory_planner7(self,angle,D,L):
        
        steering_angle = math.atan(L*angle/D)

        return steering_angle



    #@njit(fastmath=False, cache=True)
    def angle_difference(self,v1,v2):
        unit_v1 = v1/np.linalg.norm(v1)
        unit_v2 = v2/np.linalg.norm(v2)
        dot_product = np.dot(unit_v1, unit_v2)
        dot_product = max(-1,min(dot_product,1))
       # print(dot_product)
        angle = math.acos(dot_product)
        return angle


    #@njit(fastmath=False, cache=True)
    def find_closest_point(self,p,array):
        track_error_list = []
        for p_array in array:
            track_error = np.linalg.norm(p-p_array)
            track_error_list.append(track_error)

        cross_track_error = min(track_error_list)
        closest_point = array[track_error_list.index(cross_track_error)]
        
        return closest_point, cross_track_error

    
    #@njit(fastmath=False, cache=True)
    def vector_rotation(self,vector,rot_angle):
        R = np.array([[math.cos(rot_angle) , -math.sin(rot_angle)],[math.sin(rot_angle) , math.cos(rot_angle)]])
        rotated_vector = R@vector
        return rotated_vector


    #@njit(fastmath=False, cache=True)
    def cross(self,a,b):
        v1 = np.zeros(3)
        v2 = np.zeros(3)
        v1[:2] = a
        v2[:2] = b
        return np.cross(v1,v2)[2]

    #@njit(fastmath=False, cache=True)
    def turn_detection4(self,X_init,Y_init,phi_init,array,dis,num,offset,weight):
        #array = array[:,0:2]

        p = np.array([X_init,Y_init])
        m1 = np.array([0.0,0.0])   #[-1.0,0.0]
        m2 = np.array([2.,0.0])    #[2.0,0.0]
        m1_R = self.vector_rotation(m1,phi_init)
        m2_R = self.vector_rotation(m2,phi_init)
        p1 = p + m1_R
        p2 = p + m2_R
        
        angle_offset = (2*0.5/(1+np.exp(-offset*weight)))-0.5
      

        vec_trans = np.array([dis,0.])      # for moving points only

        p1_array,error1 = self.find_closest_point(p1,array)

        ### Loop Start Here ###
        phi = phi_init
        angle_sum = 0. + angle_offset
        for i in range(num):
            p2_array,error2 = self.find_closest_point(p2,array)

            v1 = p2 - p1   # from the car
            v2 = p2_array - p1_array # from the track

            if np.array_equal(p1_array,p2_array):
                break
        
            angle_diff = self.angle_difference(v1,v2)
        
            sign = np.sign(self.cross(v1,v2))
            if sign == 0:
                sign = 1.
            
            angle_signed = sign*angle_diff   # check HERE 
            angle_sum = angle_sum + angle_signed

            phi = phi+angle_signed  # correction
            vec_go = self.vector_rotation(vec_trans,phi)   # correction
            p1 = p2
            p2 = p2+vec_go
            p1_array = p2_array
        
        return angle_sum    # CHANGE HERE TO re1


    def execute(self,currState,center_lane_array,offset,vx,weight):
        # TODO Add your controller code      
        L = 1.4351*2
   
        angle = self.turn_detection4(currState[0][0],currState[0][1],currState[1][2],center_lane_array,1,3,offset,weight)   #1. , 7
        
        steering_angle = self.trajectory_planner7(angle,3.5,L) #1.5  3
      
    
        #### Administer commands ####
        current_vx = np.sqrt(currState[2][0]**2 + currState[2][1]**2)
        vx_error = vx - current_vx
        vx = vx + vx_error*2

        newAckermannCmd = AckermannDrive()
        newAckermannCmd.steering_angle = steering_angle
        if vx >= 0:
            newAckermannCmd.speed = vx
        else:
            newAckermannCmd.acceleration = -20
            newAckermannCmd.speed = 0
        
        self.controlPub.publish(newAckermannCmd)
        
        
                

class VehicleDecision():
    def __init__(self, role_name='ego_vehicle'):
        self.role_name = role_name
        self.subLaneMarker = rospy.Subscriber("/carla/%s/lane_markers"%role_name, LaneInfo, self.lanemarkerCallback)

    def find_closest_point(self,p,array):
        track_error_list = []
        for p_array in array:
            track_error = np.linalg.norm(p-p_array)
            track_error_list.append(track_error)

        cross_track_error = min(track_error_list)
        #closest_point = array[track_error_list.index(cross_track_error)]
        index = track_error_list.index(cross_track_error)
        return cross_track_error, index

    def angle_difference(self,v1,v2):
        unit_v1 = v1/np.linalg.norm(v1)
        unit_v2 = v2/np.linalg.norm(v2)
        dot_product = np.dot(unit_v1, unit_v2)
        dot_product = max(-1,min(dot_product,1))
       # print(dot_product)
        angle = math.acos(dot_product)
        return angle

    def lanemarkerCallback(self, data):
        waypoint_num = len(data.lane_markers_center.location)
        center_lane_ob = data.lane_markers_center.location
        self.lane_state = data.lane_state
        self.center_lane_array = []
       
        for i in range(0,waypoint_num):
            self.center_lane_array.append([center_lane_ob[i].x,center_lane_ob[i].y])
               
        self.center_lane_array = np.array(self.center_lane_array)
        self.lane_array = self.center_lane_array

    def obstacle_avoidance(self,obstacleList,lane_array,X,Y,vx,vy,lane_state):
        lane_vector = lane_array[18]-lane_array[15]
        if lane_state == 4:
            ego_location = np.array([X+vx*0.15,Y+vy*0.15])  #0.21
        else:
            ego_location = np.array([X,Y])  #0.21
        ego_vector = ego_location - lane_array[0]
        ego_track_error , ego_index = self.find_closest_point(ego_location,lane_array)
        ego_sign = np.sign(np.cross(ego_vector,lane_vector))
        ego_track_error = float(ego_track_error*ego_sign)
     
        obstacle_num = len(obstacleList)
      #  print(obstacle_num)
        if obstacle_num == 0:
            offset = ego_track_error
            vx = 9.5  #9.5
            weight = 0.2
        else:
            
            
            b = []
            L = []
            track_error_list = []
            y = []
            for i in range(0,obstacle_num):
                if len(obstacleList[i].vertices_locations) == 0:
                    obstacle_num = obstacle_num-1
                    continue
                obstacle_location = np.array([obstacleList[i].location.x,obstacleList[i].location.y])
                if np.linalg.norm(ego_location-obstacle_location) >= 22:  #20
                    obstacle_num = obstacle_num-1
                    continue  
                
                v1 = np.array([obstacleList[i].vertices_locations[0].vertex_location.x,obstacleList[i].vertices_locations[0].vertex_location.y])
                v2 = np.array([obstacleList[i].vertices_locations[2].vertex_location.x,obstacleList[i].vertices_locations[2].vertex_location.y])
                v3 = np.array([obstacleList[i].vertices_locations[4].vertex_location.x,obstacleList[i].vertices_locations[4].vertex_location.y])
                obstacle_orientation_vector = v3 - v1
                orientation = self.angle_difference(lane_vector,obstacle_orientation_vector)
                
                width = np.linalg.norm(v1-v2)
                length = np.linalg.norm(v1-v3)
                W = np.abs(width*math.cos(orientation))+np.abs(length*math.sin(orientation))
                L.append(float(length))
                b.append(float(W))
           

                track_error, obs_index = self.find_closest_point(obstacle_location,lane_array)
                
                if obs_index == (len(lane_array)-1):
                    over_vector = obstacle_location-lane_array[-1]
                    over_angle = self.angle_difference(over_vector,lane_vector)
                    track_error = abs(track_error*math.sin(over_angle))
                y.append(abs(obs_index-ego_index)*0.5)
                obstacle_vector = obstacle_location - lane_array[0]
                sign = np.sign(np.cross(obstacle_vector,lane_vector))
                track_error = track_error*sign
                
                
                if (orientation > 1.4)&(length<1):
                    heading = np.sign(np.cross(obstacle_orientation_vector,lane_vector))
                    if ((heading > 0)&(ego_track_error>track_error))|((heading < 0)&(ego_track_error<track_error)):
                        track_error = -track_error*0.3+ego_track_error
                   
                
                track_error_list.append(float(track_error))

           
            if obstacle_num != 0:
                mip = Model()
                mip.verbose = 0
                safe_dis = 0.4 #0.4
                safe_cross = 3   #6.5 4 3 3.5
                ego_width = 2
                ego_length = 4
                M = 10000
                m = -10000
                d = mip.add_var(var_type=CONTINUOUS,lb=-1000) 
                s = [mip.add_var(var_type=CONTINUOUS) for i in range(obstacle_num)]
                delta2 = [mip.add_var(var_type=BINARY) for i in range(obstacle_num)]
                z = [mip.add_var(var_type=CONTINUOUS,lb=-1000) for i in range(obstacle_num)]
                t = mip.add_var(var_type=CONTINUOUS) 
                delta1 = [mip.add_var(var_type=BINARY) for i in range(obstacle_num)]
                delta3 = [mip.add_var(var_type=BINARY) for i in range(obstacle_num)]
                #print(L)
                mip += d<=t
                mip += -d<=t
                for i in range(0,obstacle_num):
                    mip += ego_track_error+d-track_error_list[i]+M*delta1[i]>=safe_dis+b[i]*0.5+ego_width*0.5-s[i]
                    mip += -(ego_track_error+d-track_error_list[i])+M*(1-delta1[i])>=safe_dis+b[i]*0.5+ego_width*0.5-s[i]

                    mip += y[i]-0.5*ego_length-0.5*L[i]-safe_cross<=M*(1-delta2[i])
                    mip += y[i]-0.5*ego_length-0.5*L[i]-safe_cross>=m*delta2[i]


                    mip += ego_track_error-track_error_list[i]<=M*(1-delta3[i])
                    mip += ego_track_error-track_error_list[i]>=m*delta3[i]

                    mip += (m-M)*delta2[i]+z[i]<=track_error_list[i]
                    mip += (m-M)*delta2[i]-z[i]<=-track_error_list[i]
                    mip += (m-M)*(1-delta2[i])+z[i]<=ego_track_error+d
                    mip += (m-M)*(1-delta2[i])-z[i]<=-ego_track_error-d

                    mip += z[i]-track_error_list[i]<=M*(1-delta3[i])
                    mip += z[i]-track_error_list[i]>=m*delta3[i]

                    s[i] <= safe_dis-0.2

                if lane_state == 3:
                    mip += d+ego_track_error <= 1
                elif lane_state == 5:
                    mip += -(d+ego_track_error) <= 1
                
                mip.objective = minimize(t+xsum(M*s[i] for i in range(obstacle_num)))
                status = mip.optimize(max_seconds=300)
                offset = -mip.vars[0].x

                s_list = []
                for i in range(obstacle_num):
                    s_list.append(mip.vars[1+i].x)

                if sum(s_list)==0:
                    vx = 8.5  #7.5 9
                    weight = 1
                else:
                    vx = 5
                    weight = 2
                

            else:
                offset = ego_track_error
                #print(offset)
                vx = 9  #8.7
                weight = 0.2
            
      #  time.sleep(0.4)   #0.15
        return offset,vx,weight  ## Change to Offset pls!!!!!


    # TODO Add your decision logic here

class VehiclePerception:
    def __init__(self, role_name='ego_vehicle'):
        self.locationSub = rospy.Subscriber("/carla/%s/location"%role_name, LocationInfo, self.locationCallback)
        self.obstacleSub = rospy.Subscriber("/carla/%s/obstacles"%role_name, ObstacleList, self.obstacleCallback)
        self.position = None
        self.velocity = None 
        self.rotation = None
        self.obstacleList = None

        
    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x), np.radians(data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles
        
        
        


def run_model(role_name):
    
    rate = rospy.Rate(100)  # 100 Hz    

    perceptionModule = VehiclePerception(role_name=role_name)
    decisionModule = VehicleDecision(role_name)
    controlModule = VehicleController(role_name=role_name)

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    print("Starter code is running")

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        obstacleList = perceptionModule.obstacleList
        
        

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        lane_array = decisionModule.lane_array
        lane_state = decisionModule.lane_state
        
        offset,vx,weight = decisionModule.obstacle_avoidance(obstacleList,lane_array,currState[0][0],currState[0][1],currState[2][0],currState[2][1],lane_state)
        
       # print(offset)
        
        if not currState or not currState[0]:
            continue
        
        # Execute 
        controlModule.execute(currState,lane_array,offset,vx,weight)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Running vechile")

    role_name_default = 'ego_vehicle'

    parser.add_argument('--name', type=str, help='Rolename of the vehicle', default=role_name_default)
    argv = parser.parse_args()
    role_name = argv.name
    rospy.init_node("baseline")
    role_name = 'hero0'
    try:
        run_model(role_name_default)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
    