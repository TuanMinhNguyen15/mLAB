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
import osqp
from scipy import sparse
import scipy as sp
import math
from scipy.linalg import expm
from scipy.signal import cont2discrete
from scipy import spatial
from scipy.linalg import block_diag
from mip import *
#from numba import njit , jit
####################
start_time = time.time() 
prev_phi = 0 

class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.role_name = role_name
        self.controlPub = rospy.Publisher("/carla/%s/ackermann_control"%role_name, AckermannDrive, queue_size = 1)


    #@njit(fastmath=False, cache=True)
    def nonlinear_dynamics(self,x1,x2,x3,x4,x5,x6,u1,u2):
        ##### param #####
        Df = 0.17
        Cf = 1
        Bf = 2.58
        Dr = 0.19
        Br = 3.38
        Cm1 = 500
        Cm2 = 0.0545
        Cr = 1
        Cd = 0.001

        m = 1300
        Iz = 16.35
        g = 9.81 
        h = 0.16764
        lf = 1.4351
        lr = 1.4351
        u_fr = 1.0489
        ################
        x_dot = np.array([x4*math.cos(x3)-x5*math.sin(x3),
                            x5*math.cos(x3)+x4*math.sin(x3),
                            x6,
                            -(Cd*x4**2 - u2/3.5*(Cm1 - Cm2*x4) + Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.sin(u1) - m*x5*x6 + (g*lf*m*u_fr)/(lf + lr))/(m*((h*u_fr)/(lf + lr) + 1)),
                            -(Dr*math.sin(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))) - Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.cos(u1) + m*x4*x6)/m,
                            (Dr*lr*math.sin(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))) + Df*lf*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.cos(u1))/Iz])
        return x_dot

    #@njit(fastmath=False, cache=True)
    def nonlinear_discretize(self,x1,x2,x3,x4,x5,x6,u1,u2,Ts):
        x_k = np.array([x1,x2,x3,x4,x5,x6])
        x_kplus1 = x_k + Ts*self.nonlinear_dynamics(x1,x2,x3,x4,x5,x6,u1,u2)
        return x_kplus1


    #@njit(fastmath=False, cache=True)
    def linearize(self,x1,x2,x3,x4,x5,x6,u1,u2):
        ##### param #####
        Df = 0.17
        Cf = 1
        Bf = 2.58
        Dr = 0.19
        Br = 3.38
        Cm1 = 500
        Cm2 = 0.0545
        Cr = 1
        Cd = 0.001

        m = 1300
        Iz = 16.35
        g = 9.81 
        h = 0.16764
        lf = 1.4351
        lr = 1.4351
        u_fr = 1.0489
        ################
        Ac = np.array([[ 0., 0., - x5*math.cos(x3) - x4*math.sin(x3), math.cos(x3),  -math.sin(x3), 0],
                        [ 0., 0.,   x4*math.cos(x3) - x5*math.sin(x3),  math.sin(x3), math.cos(x3),  0],
                        [ 0., 0., 0., 0., 0., 1.],
                        [ 0., 0., 0., -(Cm2*u2/3.5 + 2*Cd*x4 + (Bf*Cf*Df*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*(x5 + lf*x6))/(x4**2*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/(m*((h*u_fr)/(lf + lr) + 1)), (m*x6 + (Bf*Cf*Df*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/(m*((h*u_fr)/(lf + lr) + 1)), (m*x5 + (Bf*Cf*Df*lf*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/(m*((h*u_fr)/(lf + lr) + 1))],
                        [ 0., 0., 0.,  ((Br*Cr*Dr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4)))*(x5 - lr*x6))/(x4**2*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) - m*x6 + (Bf*Cf*Df*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*(x5 + lf*x6))/(x4**2*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/m,  -((Br*Cr*Dr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) + (Bf*Cf*Df*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/m, -(m*x4 - (Br*Cr*Dr*lr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) + (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/m],
                        [ 0., 0., 0., -((Br*Cr*Dr*lr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4)))*(x5 - lr*x6))/(x4**2*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) - (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*(x5 + lf*x6))/(x4**2*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/Iz, ((Br*Cr*Dr*lr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) - (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/Iz,   -((Br*Cr*Dr*lr**2*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) + (Bf*Cf*Df*lf**2*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/Iz]])
        
        Bc = np.array([[ 0., 0.],
                        [ 0., 0.],
                        [ 0., 0.],
                        [ -(Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.cos(u1) + (Bf*Cf*Df*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1))/(m*((h*u_fr)/(lf + lr) + 1)), (Cm1 - Cm2*x4)/(m*((h*u_fr)/(lf + lr) + 1))],
                        [ -(Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.sin(u1) - (Bf*Cf*Df*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1))/m, 0],
                        [ -(Df*lf*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.sin(u1) - (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1))/Iz, 0]])
        return Ac,Bc

    #@njit(fastmath=False, cache=True)
    def discretize(self,Ac,Bc,Ts):
        Ad = np.eye(Ac.shape[0]) + Ts*Ac
        Bd = Ts*Bc
        return Ad,Bd


    def QP_solver8(self,Ad_seq,Bd,Q,s,QN,R,x0,xr_seq,ur,umin,umax,u0,delta_u,M,N_plus_W,N_minus_W,N,Ts):
        [nx, nu] = Bd.shape 

        # Quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R),sparse.kron(sparse.eye(N), s)], format='csc')
        # Linear objective
        xr_stage = xr_seq[0:N]
        xr_N = xr_seq[N]
        q = np.hstack([(((-Q.dot(xr_stage.T))).T).reshape(-1), -QN.dot(xr_N),
                    np.kron(np.ones(N), -R.dot(ur)),np.zeros(N)])
        
        
        
        # Linear dynamics

        diag_As = block_diag(*Ad_seq)
        G = np.zeros([nx*(N+1),nx*(N+1)])
        G[nx:diag_As.shape[0]+nx,:diag_As.shape[1]] = diag_As
        K = np.kron(np.eye(N+1),-np.eye(nx))
        Ax = G + K
        Ax = sparse.csc_matrix(Ax)

        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)

        S = sparse.csc_matrix((nx*(N+1), N))
        Aeq = sparse.hstack([Ax, Bu , S])

        ## Linearized system dynamics constraints STACKING ##
        stacking_list = []
        for k in range(0,N):
            stacking_list.append(Ad_seq[k]@xr_seq[k] + Bd@ur - self.nonlinear_discretize(*xr_seq[k],*ur,Ts))
        linearize_constraint = np.array(stacking_list).reshape(-1)
        
        #####################################################

        leq = np.hstack([-x0, linearize_constraint])
        ueq = leq

        # Input and state constraints
        Aineq = sparse.hstack([sparse.csc_matrix((N*nu, (N+1)*nx)),sparse.eye(N*nu),sparse.csc_matrix((N*nu, N))])
        lineq = np.kron(np.ones(N), umin)
        uineq = np.kron(np.ones(N), umax)

        # Halfspace Constraints
        half_up = np.hstack([np.zeros((N,nx)),block_diag(*M),np.zeros((N,N*nu)),-np.eye(N)])
        half_down = np.hstack([np.zeros((N,nx)),block_diag(*M),np.zeros((N,N*nu)),np.eye(N)])
        A_half = np.vstack([half_up,half_down])

        l_half = np.hstack([np.kron(np.ones(N),-math.inf),N_minus_W])
        u_half = np.hstack([N_plus_W,np.kron(np.ones(N),math.inf)])

        # Slack Constraints
        A_slack = np.hstack([np.zeros((N,(N+1)*nx)),np.zeros((N,N*nu)),np.eye(N)])

        l_slack = np.zeros(N)
        u_slack = np.kron(np.ones(N),math.inf)

        # Input Rate of Change Contraints
        A_delta = np.hstack([np.zeros((N*nu,(N+1)*nx)),np.kron(np.eye(N),np.eye(nu))+np.kron(np.eye(N,k=-1),-np.eye(nu)),np.zeros((N*nu,N))])

        l_delta = np.hstack([u0,np.kron(np.ones(N-1),-delta_u)])
        u_delta = np.hstack([u0,np.kron(np.ones(N-1),delta_u)])

        # OSQP constraints
        A = sparse.vstack([Aeq, Aineq, A_half, A_slack, A_delta], format='csc')
        l = np.hstack([leq, lineq, l_half, l_slack, l_delta])
        u = np.hstack([ueq, uineq, u_half, u_slack, u_delta])

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace
        prob.setup(P, q, A, l, u, warm_start=True,verbose = False)

        # Solving QP 
        results = prob.solve()

        return results.x

    #@njit(fastmath=False, cache=True)
    def M_N_W(self,trajectory,C2,C3,dl,dr):
        X = trajectory[:,0]
        Y = trajectory[:,1]
        
        M = []
        N_plus_W = []
        N_minus_W = []
        for i in range(1,len(X)):
            Mi = np.array([(X[i]-C2) , (Y[i]-C3) , 0. , 0. , 0. , 0.])
            Ni = (X[i]-C2)*X[i] + (Y[i]-C3)*Y[i]
            Wi_plus = dr*math.sqrt((X[i]-C2)**2 + (Y[i]-C3)**2)
            Wi_minus = dl*math.sqrt((X[i]-C2)**2 + (Y[i]-C3)**2)
            M.append(Mi)
            N_plus_W.append(Ni+Wi_plus)
            N_minus_W.append(Ni-Wi_minus)
        
        return M,N_plus_W,N_minus_W

    #@njit(fastmath=False, cache=True)
    def trajectory_generator(self,vx,delta,X_init,Y_init,phi_init,L,Ts,N):
        C1 = phi_init 
        #print(delta)
        C2 = X_init - L*math.sin(C1)/math.tan(delta)
        C3 = Y_init + L*math.cos(C1)/math.tan(delta)
        phi_dot = (vx/L)*math.tan(delta)
        trajectory = []
        for k in range(0,N):
            phi_k = (vx/L)*math.tan(delta)*Ts*k + C1
            X_k = L*math.sin(phi_k)/math.tan(delta) + C2
            Y_k = -L*math.cos(phi_k)/math.tan(delta) + C3
            trajectory.append(np.array([X_k,Y_k,phi_k,vx,0,phi_dot,delta,C2,C3]))   # just added N here!!!!
        return trajectory


    #@njit(fastmath=False, cache=True)
    def trajectory_planner7(self,vx,angle,D,X_init,Y_init,phi_init,L,Ts,N):
        
        steering_angle = math.atan(L*angle/D)
        #steering_angle = angle
        #print(steering_angle)
        """
        #steering_angle2 = math.atan(distance/5)
        steering_angle2 = 0.2*distance**3
        steering_angle = 1.0*steering_angle1 + 1.0*steering_angle2
        """

        trajectory_chosen = self.trajectory_generator(vx,steering_angle,X_init,Y_init,phi_init,L,Ts,N)
        return trajectory_chosen

    def trajectory_linearization(self,trajectory_chosen,u1,u2,Ts):
        trajectory_chosen = trajectory_chosen[:trajectory_chosen.shape[0]-1,:]   # No need to linearize at the last waypoint
        Ad_seq = []
        for x_op in trajectory_chosen:
            Ac,Bc = self.linearize(*x_op,u1,u2)
            Ad , Bd = self.discretize(Ac,Bc,Ts)
            Ad_seq.append(Ad)

        return Ad_seq , Bd


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
    def turn_detection4(self,X_init,Y_init,phi_init,array,dis,num,offset):
        #array = array[:,0:2]

        p = np.array([X_init,Y_init])
        m1 = np.array([0.0,0.0])   #[-1.0,0.0]
        m2 = np.array([2.,0.0])    #[2.0,0.0]
        m1_R = self.vector_rotation(m1,phi_init)
        m2_R = self.vector_rotation(m2,phi_init)
        p1 = p + m1_R
        p2 = p + m2_R
        
        ##track_error = self.cross_track_error(X_init,Y_init,phi_init,array) #- offset   # offet placed here
      
     ##   track_error = offset
       
       # print(track_error)
       # track_error = max(-2,min(track_error,2))
       # angle_offset = 0.05*track_error**3   #1
        angle_offset = (2*0.3/(1+np.exp(-offset)))-0.3
      

        vec_trans = np.array([dis,0.])      # for moving points only

        """
        # vec_go = vector_rotation(vec_trans,phi_init-angle_offset)   # re-orientation
        vec_go = vector_rotation(vec_trans,phi_init)   # re-orientation
        
        p1 = np.array([X_init,Y_init])
        p2 = p1 + vec_go
        """
        p1_array,error1 = self.find_closest_point(p1,array)

       # print(p1)
        ### Loop Start Here ###
        phi = phi_init
        angle_sum = 0. + angle_offset
        for i in range(num):
            p2_array,error2 = self.find_closest_point(p2,array)
          #  print(p2)


            v1 = p2 - p1   # from the car
            v2 = p2_array - p1_array # from the track

            if np.array_equal(p1_array,p2_array):
                print('cri')
                print(i)
                re1 = angle_offset
                break

            angle_diff = self.angle_difference(v1,v2)
        
            sign = np.sign(self.cross(v1,v2))
            if sign == 0:
                sign = 1.
            
            angle_signed = sign*angle_diff   # check HERE 
            error2 = sign*error2
            
            angle_sum = angle_sum + angle_signed
           # print(angle_sum)
            
        
            if i == 2:   #2
                re1 = angle_sum

            phi = phi+angle_signed  # correction
            vec_go = self.vector_rotation(vec_trans,phi)   # correction
            p1 = p2
            p2 = p2+vec_go
            p1_array = p2_array
        # p_list.append(p2)
       # print('-----------')  
        print(angle_sum)
        if np.abs(angle_sum) > 0.3:
            vx = 5   #10
            re1 = re1 #- angle_offset
        else:
            vx = 15    #8
        
        return vx,re1    # CHANGE HERE TO re1

    #@njit(fastmath=False, cache=True)
    def cross_track_error(self,X_init,Y_init,phi_init,array):
        l = np.array([0.,0.1])
        p = np.array([X_init,Y_init])
        r = np.array([0.,-0.1])

        #R = np.array([[math.cos(phi_init) , -math.sin(phi_init)],[math.sin(phi_init) , math.cos(phi_init)]])
        l_Rot = self.vector_rotation(l,phi_init)
        r_Rot = self.vector_rotation(r,phi_init)

        l_trans = p + l_Rot
        r_trans = p + r_Rot

        p_closest, track_error = self.find_closest_point(p,array)

        dis_l = np.linalg.norm(l_trans - p_closest)
        dis_r = np.linalg.norm(r_trans - p_closest)

        track_error = np.sign(dis_r-dis_l)*track_error
        track_error = track_error     # offset can be placed here
        return track_error

    def execute(self,currState,center_lane_array,offset):
        # TODO Add your controller code
        #### Getting current state and center_lane_array ####
        global start_time
        global prev_phi

        current_time = time.time()
        time_elapse = current_time - start_time

        phi = currState[1][2]
       
        phi_dot = (phi - prev_phi)/time_elapse
        prev_phi = phi
        start_time = current_time
        time.sleep(0.2)   # add a little bit of delay

        current_state = np.array([currState[0][0],currState[0][1],phi,currState[2][0],currState[2][1],phi_dot])
        #### Algorithm Start Here ####
        N = 5
        Ts = 0.03
        nx = 6
        nu = 2
        ns = 1
        dl = 0.01
        dr = 0.01

        lf = 1.4351
        lr = 1.4351



        Q = np.diag([10, 10, 1, 10, 1, 1.])
        QN = Q
        R = np.diag([200, 200])
        s = 500

        umin = np.array([-0.54906585,-1.5]) 
        umax = np.array([0.54906585,100]) 
        delta_u = np.array([0.7,10])
        
        u0 = np.array([0,0])
        
        
        vel,angle = self.turn_detection4(current_state[0],current_state[1],current_state[2],center_lane_array,1,5,offset)   #1. , 7
        
        trajectory_chosen = np.array(self.trajectory_planner7(vel,angle,3.3,current_state[0],current_state[1],current_state[2],lf+lr,Ts,N)) #1.5  3
        C2 = trajectory_chosen[0,7]
        C3 = trajectory_chosen[0,8]
        

        u2 = trajectory_chosen[0,3]
        u1 = trajectory_chosen[0,6]
        ur = np.array([u1,u2])
        
        trajectory_chosen = trajectory_chosen[:,:trajectory_chosen.shape[1]-3]  ## Getting rid of the steering angle, C1 , C2 as the states !!!!
        M,N_plus_W,N_minus_W = self.M_N_W(trajectory_chosen,C2,C3,dl,dr)
        Ad_seq , Bd = self.trajectory_linearization(trajectory_chosen,*ur,Ts)
        re = self.QP_solver8(Ad_seq,Bd,Q,s,QN,R,current_state,trajectory_chosen,ur,umin,umax,u0,delta_u,M,N_plus_W,N_minus_W,N-1,Ts) #N_here = N-1\
        vx_new = re[(N)*nx+nu+1]
        steering_angle_new = re[(N)*nx+nu]
        #print(vx_new)
        #### Administer commands ####
        
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = vx_new
        newAckermannCmd.steering_angle = steering_angle_new
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
        
        return cross_track_error

    def lanemarkerCallback(self, data):
        waypoint_num = len(data.lane_markers_right.location)
        center_lane_ob = data.lane_markers_center.location
        right_lane_ob = data.lane_markers_right.location
        left_lane_ob = data.lane_markers_left.location
        self.lane_state = data.lane_state
        self.center_lane_array = []
        self.left_lane_array = []
        self.right_lane_array = []
        
        for i in range(0,waypoint_num):
            self.center_lane_array.append([center_lane_ob[i].x,center_lane_ob[i].y])
            self.right_lane_array.append([right_lane_ob[i].x,right_lane_ob[i].y])
            self.left_lane_array.append([left_lane_ob[i].x,left_lane_ob[i].y])
          
        self.center_lane_array = np.array(self.center_lane_array)
        self.left_lane_array = np.array(self.left_lane_array)
        self.right_lane_array = np.array(self.right_lane_array)
        self.right_point = self.right_lane_array[0]
        self.left_point = self.left_lane_array[0]

        """
        if data.lane_state == 3:
            self.lane_array = self.right_lane_array
        elif data.lane_state == 4:
            self.lane_array = self.center_lane_array
        else:
            self.lane_array = self.left_lane_array
        """
        self.lane_array = self.center_lane_array

    def obstacle_avoidance(self,obstacleList,lane_array,right_point,left_point,X,Y,lane_state):
        lane_vector = lane_array[9]-lane_array[0]
        ego_location = np.array([X,Y])
        ego_vector = ego_location - lane_array[0]
        ego_track_error = self.find_closest_point(ego_location,lane_array)
        ego_sign = np.sign(np.cross(ego_vector,lane_vector))
        #ego_right_dis = np.linalg.norm(ego_location-right_point)
        #ego_left_dis = np.linalg.norm(ego_location-left_point)
        #ego_sign = np.sign(ego_right_dis - ego_left_dis)
        ego_track_error = ego_track_error*ego_sign
       # print(ego_track_error)
        obstacle_num = len(obstacleList)
        if obstacle_num == 0:
            if np.abs(ego_track_error)<=0.4:
                ego_track_error = 0
            offset = ego_track_error
        else:
            
            obstacle_location_array = []
            for i in range(0,obstacle_num):
                obstacle_location_array.append([obstacleList[i].location.x,obstacleList[i].location.y])

            obstacle_location_array = np.array(obstacle_location_array)
            track_error_list = []
            b = []
            for i in range(0,obstacle_num):
                track_error = self.find_closest_point(obstacle_location_array[i],lane_array)
                #right_dis = np.linalg.norm(obstacle_location_array[i]-right_point)
                #left_dis = np.linalg.norm(obstacle_location_array[i]-left_point)
                obstacle_vector = obstacle_location_array[i] - lane_array[0]
                #sign = np.sign(right_dis - left_dis)
                sign = np.sign(np.cross(obstacle_vector,lane_vector))
                track_error = track_error*sign
                track_error_list.append(track_error)

           # print(track_error_list)
            mip = Model()
            mip.verbose = 0
            safe_dis = 2.5 #2.5
            M = 10000
            d = mip.add_var(var_type=CONTINUOUS,lb=-1000) 
            t = mip.add_var(var_type=CONTINUOUS) 
            delta = [mip.add_var(var_type=BINARY) for i in range(obstacle_num)]

            mip += d<=t
            mip += -d<=t
            for i in range(0,obstacle_num):
                mip += ego_track_error+d-track_error_list[i]+M*delta[i]>=safe_dis
                mip += -(ego_track_error+d-track_error_list[i])+M*(1-delta[i])>=safe_dis

            if lane_state == 3:
                mip += d+ego_track_error <= 0
            elif lane_state == 5:
                mip += -(d+ego_track_error) <= 0
            
            mip.objective = minimize(t)
            status = mip.optimize(max_seconds=300)
            offset = -mip.vars[0].x
      #  print(offset)

        return offset  ## Change to Offset pls!!!!!


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
        #obstacle_location_array = np.array([])
        obstacleList = perceptionModule.obstacleList
        
        

        # Get the current position and orientation of the vehicle
        currState =  (perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity)
        lane_array = decisionModule.lane_array
        lane_state = decisionModule.lane_state
        right_point = decisionModule.right_point
        left_point = decisionModule.left_point
        
        offset = decisionModule.obstacle_avoidance(obstacleList,lane_array,right_point,left_point,currState[0][0],currState[0][1],lane_state)
        
       # print(offset)
        
        if not currState or not currState[0]:
            continue
        
        # Execute 
        controlModule.execute(currState,lane_array,offset)

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
    