#!/usr/bin/env python

# license removed for brevity

import rospy
import rospkg
import random
import time
import numpy 		 as np
import scipy.linalg 	 as la
import numpy.linalg 	 as lalg

from math 		 import *
from numpy 		 import sin, cos, tan
from std_msgs.msg 	 import String
from geometry_msgs.msg 	 import Twist
from sensor_msgs.msg   import Imu
from turtlesim.msg 	 import Pose
from turtle_files.msg 	 import *
from waypoint_manager.msg        import bot_states

# from turtle_files.msg 	 import state_est
# from turtle_files.msg 	 import sensor_op
# from turtle_files.msg 	 import param_turt_ekf
# from geometry_msgs.msg 	 import PoseStamped 

################## Bot State Initialization ##################

RunCase = 4

if RunCase == 1:
    # Case 1: 4 LM Zig-Zag Actual LM

    x_true       =  0.0            # Meters
    y_true       = -2.0            # Meters
    psi_true     =  90*(np.pi/180) # Radians

    x_est_init   =  0.01           # Meters
    y_est_init   = -2.0            # Meters
    psi_est_init =  90*(np.pi/180) # Radians


elif RunCase == 2:
    # Case 2: 9 LM Zig-Zag (Ran on 24, Aug 2019) Simulated LM

    x_true       =  0.0            # Meters
    y_true       = -6.0            # Meters
    psi_true     =  90*(np.pi/180) # Radians

    x_est_init   =  0.0            # Meters
    y_est_init   = -6.01           # Meters
    psi_est_init =  90*(np.pi/180) # Radians


elif RunCase == 3:
    # Case 3: TSP Case (24 Apr, 2019) Simulated LM

    x_true       = -2.0            # Meters
    y_true       =  1.0            # Meters
    psi_true     =  0*(np.pi/180)  # Radians

    x_est_init   = -2.01           # Meters
    y_est_init   =  1.0            # Meters
    psi_est_init =  0*(np.pi/180)  # Radians


elif RunCase == 4:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM

    x_true       = -1.0            # Meters
    y_true       = -3.0            # Meters
    psi_true     = -10*(np.pi/180) # Radians

    x_est_init   = -1.0            # Meters
    y_est_init   = -3.01           # Meters
    psi_est_init = -10*(np.pi/180) # Radians

elif RunCase == 5:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM

    x_true       = -1.0            # Meters
    y_true       = -2.0            # Meters
    psi_true     =  0*(np.pi/180) # Radians

    x_est_init   = -1.0            # Meters
    y_est_init   = -2.01           # Meters
    psi_est_init =  0*(np.pi/180) # Radians

############ Velocity, Omega, Flag Initialization ############

V_in         =  0.0
w_in         =  0.0

V_est        =  0.0
w_est        =  0.0

start_flag   =  0                       # 0 - Stop, 1 - Start
update_flag  =  0			# 1 for range-only, 2 for bearing-only, 3 for both

vel_std      =  0.0			# Velocity  Noise Std
omega_std    =  0.0			# Turn-rate Noise Std

############# Landmark Parameters Initialization #############

LM_x       	     =  np.array([0.0, 0.0, 0.0])	 # LM X-axis values
LM_y      	     =  np.array([0.0, 0.0, 0.0])	 # LM Y-axis values

meas_dist    	  = -1			                 # Maximum sensor range between LM and Vehicle
num_LM       	  = -1			                 # Number of LMs
rho_sensor   	  = np.array([-100.0])	         # Sensor Range Output
br_sensor    	  = np.array([-100.0])		     # Sensor Bearing Output
sensor_flag  	  =  0			                 # Data coming from sensors.py or not (0-No, 1-Yes)

rho_sensor_fake   = np.array([-100.0])	         # Sensor Range Output
br_sensor_fake    = np.array([-100.0])		     # Sensor Bearing Output
sensor_flag_fake  =  0			                 # Data coming from sensors.py or not (0-No, 1-Yes)

################ Time for EKF  Initialization ################

t	        = time.time()	# Seconds
t_prev      = t			    # Seconds
t_init      = t             # Seconds
time_thresh = 3.0           # Seconds

##################### EKF Initialization #####################

x_hat        = np.matrix([[x_est_init], [y_est_init], [psi_est_init]])
P_hat        = np.matrix([[0.03, 0, 0], [0, 0.03, 0], [0, 0, 0.01]])

# Q            = np.matrix([[0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.001]])
Qu           = np.matrix([[0.0001, 0],[0, 0.00001]])
R_rng        = 0.08 			# Meters
R_br         = 0.10 			# Radians

num_iter     = -1 			# Prediction Iteration

########################## -------- ##########################

########## Function for Angle Wrapping ##########
def pi2pi(angle):
    while((fabs(angle)- np.pi) >0.001):
        if (angle > np.pi):
            angle = angle - 2*np.pi
        if (angle <-np.pi):
            angle = angle + 2*np.pi
    return angle


## Fcn to Check Range of  Vehicle from Each LM ##
def check_range(x,y,xLM,yLM,r):
    checkRange = 0
    dist = np.sqrt((x-xLM)*(x-xLM) + (y-yLM)*(y-yLM))
    if (dist <= r):
        checkRange = 1
    return(checkRange)


### Callback to Turtlesim /turtlebot1/cmd_vel ###
def callback1(data1):
    global V_in, V_est

    V_in  	 = data1.linear.x
    V_est 	 =  V_in # + vel_std*random.random()


### Callback to Turtlesim /turtlebot1/sensors/imu_data ###
def callback4(data4):
    global w_in, w_est

    w_in     = data4.angular_velocity.z
    w_est    =  w_in # + omega_std*random.random()

########### Callback to Sensors  Data ###########
def callback2(data2):
    global rho_sensor, sensor_flag, br_sensor
    rho_sensor   = data2.calc_range
    br_sensor    = data2.calc_bearing
    sensor_flag  = data2.flag_sensor


# ########### Callback to Sensors  Data ###########
# def callback4(data4):
#     global rho_sensor_fake, sensor_flag_fake, br_sensor_fake
#     rho_sensor_fake   = data4.calc_range
#     br_sensor_fake    = data4.calc_bearing
#     sensor_flag_fake  = data4.flag_sensor

################################## Change to Estimate States in callback3 ##########

###### Callback to Turtlesim /mocap_raw/simple ######
def callback3(data3):
    global x_true, y_true, psi_true

    x_true  	 = data3.x
    y_true  	 = data3.y
    psi_true 	 = data3.psi*(np.pi/180)
    # psi_true   = data3.theta

########## Callback for Turtlesim  EKF ##########
def callbackparam(data5):
    global LM_x, LM_y, vel_in, meas_dist, num_LM, start_flag, update_flag
    global x_est_init, y_est_init, psi_est_init, num_iter, vel_std, omega_std

    vel_in	 = data5.vel_in
    LM_x  	 = np.asarray(data5.lm_x)
    LM_y  	 = np.asarray(data5.lm_y)
    num_LM	 = data5.lm_num
    meas_dist	 = data5.lm_meas_dist

    #x_est_init	  = data5.ekf_x_init
    #y_est_init	  = data5.ekf_y_init
    #psi_est_init = data5.ekf_psi_init

    num_iter	 = data5.ekf_num_iter
    vel_std	 = data5.ekf_vel_std
    omega_std	 = data5.ekf_omega_std
    start_flag   = data5.flag_start
    update_flag  = data5.flag_update_ekf
 

###### The Extended Kalman Filter Function ######
def ekf():
    global t_prev, t     , update_flag
    global V_est , w_est , rho_sensor, br_sensor
    global x_hat , P_hat , num_iter  , Qu   , R_rng, R_br
    global x_true, y_true, num_LM    , LM_x, LM_y , meas_dist

    t      = rospy.get_time()
    dt     = t - t_prev
    t_prev = t

    # print 'dt = ', dt
    # print update_flag

    for i in xrange(num_iter):

        c_phi   = cos(x_hat.item(2))
        s_phi   = sin(x_hat.item(2))

        x_dot   = np.matrix([[V_est*c_phi],[V_est*s_phi],[w_est]])

        x_hat   = x_hat + np.dot(x_dot, np.divide(dt,num_iter))
        x_hat[2] = pi2pi(x_hat.item(2))

        A	= np.matrix([[0, 0, -V_est*c_phi],[0, 0, V_est*s_phi],[0, 0, 0]])
        B   = np.matrix([[c_phi, 0], [s_phi, 0], [0, 1]])

        Q   = np.dot(np.dot(B,Qu),np.transpose(B))

        P_temp  = np.dot(A, P_hat) + np.dot(P_hat, np.transpose(A)) + Q
        P_hat   = P_hat + np.dot(P_temp, np.divide(dt,num_iter))

    if (sensor_flag == 1) and (t - t_init >= time_thresh):			#FIXME
        # print "sensor_flag =", sensor_flag  

        if ((update_flag == 1) or (update_flag == 3)): 

            # print 'range'
            for j in xrange(num_LM):

                # chk_rng = check_range(rho_sensor[j], meas_dist)

                if (rho_sensor[j] >= 0):

                    # print rho_sensor, br_sensor, j

                    xV       = x_hat.item(0)
                    yV       = x_hat.item(1)
                    xLM      = LM_x.item(j)
                    yLM      = LM_y.item(j)

                    rho_est  = np.sqrt((xV - xLM)*(xV - xLM) + (yV - yLM)*(yV - yLM))

                    hr1      = (xV - xLM)/rho_est
                    hr2      = (yV - yLM)/rho_est
                    Hr       = np.matrix([hr1, hr2, 0])

                    l1       = R_rng + np.dot(Hr,np.dot(P_hat, np.transpose(Hr)))
                    l1_sclr  = l1.item(0)
                    l1_inv   = 1/l1_sclr
                    L        = l1_inv*np.dot(P_hat, np.transpose(Hr))

                    pTemp    = np.identity(3) - np.dot(L,Hr)
                    P_hat    = np.dot(pTemp,P_hat)

                    diffTmp  = rho_sensor[j] - rho_est
                    x_hat    = x_hat + np.dot(L, diffTmp)
                    x_hat[2] = pi2pi(x_hat.item(2))


        if ((update_flag == 2) or (update_flag == 3)):	### TODO ###

            # print 'bearing'
            for j in xrange(num_LM):

                #print j, br_sensor
                if (br_sensor[j] >= 0):

                    # print rho_sensor, br_sensor, j

                    xV       =  x_hat.item(0)
                    yV       =  x_hat.item(1)
                    xLM      =  LM_x.item(j)
                    yLM      =  LM_y.item(j)

                    dino     =  (xV - xLM)*(xV - xLM) + (yV - yLM)*(yV - yLM)
                    angl     =  np.arctan2(-(yV - yLM),-(xV - xLM))
                    br_est   =  pi2pi(angl - x_hat.item(2))

                    hb1      = -(yV - yLM)/dino
                    hb2      =  (xV - xLM)/dino
                    Hb       =  np.matrix([hb1, hb2, -1])

                    l1       = R_br + np.dot(Hb,np.dot(P_hat, np.transpose(Hb)))
                    l1_sclr  = l1.item(0)
                    l1_inv   = 1/l1_sclr
                    L        = l1_inv*np.dot(P_hat, np.transpose(Hb))

                    pTemp    = np.identity(3) - np.dot(L,Hb)
                    P_hat    = np.dot(pTemp,P_hat)

                    diffTmp  = pi2pi(br_sensor[j] - br_est)
                    x_hat    = x_hat + np.dot(L, diffTmp)
                    x_hat[2] = pi2pi(x_hat.item(2))


    if (sensor_flag_fake == 1):			#FIXME
        # print "sensor_flag =", sensor_flag_fake  

        if ((update_flag == 1) or (update_flag == 3)):

            # print 'range'
            for j in xrange(num_LM):

                # chk_rng = check_range(rho_sensor_fake[j], meas_dist)

                if (rho_sensor_fake[j] >= 0):

                    # print rho_sensor_fake, br_sensor, j

                    xV       = x_hat.item(0)
                    yV       = x_hat.item(1)
                    xLM      = LM_x.item(j)
                    yLM      = LM_y.item(j)

                    rho_est  = np.sqrt((xV - xLM)*(xV - xLM) + (yV - yLM)*(yV - yLM))

                    hr1      = (xV - xLM)/rho_est
                    hr2      = (yV - yLM)/rho_est
                    Hr       = np.matrix([hr1, hr2, 0])

                    l1       = R_rng + np.dot(Hr,np.dot(P_hat, np.transpose(Hr)))
                    l1_sclr  = l1.item(0)
                    l1_inv   = 1/l1_sclr
                    L        = l1_inv*np.dot(P_hat, np.transpose(Hr))

                    pTemp    = np.identity(3) - np.dot(L,Hr)
                    P_hat    = np.dot(pTemp,P_hat)

                    diffTmp  = rho_sensor_fake[j] - rho_est
                    x_hat    = x_hat + np.dot(L, diffTmp)
                    x_hat[2] = pi2pi(x_hat.item(2))


        if ((update_flag == 2) or (update_flag == 3)):	### TODO ###

            # print 'bearing'
            for j in xrange(num_LM):

                if (br_sensor_fake[j] >= 0):

                    # print rho_sensor, br_sensor_fake, j

                    xV       =  x_hat.item(0)
                    yV       =  x_hat.item(1)
                    xLM      =  LM_x.item(j)
                    yLM      =  LM_y.item(j)

                    dino     =  (xV - xLM)*(xV - xLM) + (yV - yLM)*(yV - yLM)
                    angl     =  np.arctan2(-(yV - yLM),-(xV - xLM))
                    br_est   =  pi2pi(angl - x_hat.item(2))

                    hb1      = -(yV - yLM)/dino
                    hb2      =  (xV - xLM)/dino
                    Hb       =  np.matrix([hb1, hb2, -1])

                    l1       = R_br + np.dot(Hb,np.dot(P_hat, np.transpose(Hb)))
                    l1_sclr  = l1.item(0)
                    l1_inv   = 1/l1_sclr
                    L        = l1_inv*np.dot(P_hat, np.transpose(Hb))

                    pTemp    = np.identity(3) - np.dot(L,Hb)
                    P_hat    = np.dot(pTemp,P_hat)
    
                    diffTmp  = pi2pi(br_sensor_fake[j] - br_est)
                    x_hat    = x_hat + np.dot(L, diffTmp)
                    x_hat[2] = pi2pi(x_hat.item(2))

    print 'x_true = ', x_true
    print 'x est  = ', x_hat.item(0)
    print 'y_true = ', y_true
    print 'y est  = ', x_hat.item(1)
    print 'p_true = ', pi2pi(psi_true)*(180/np.pi)
    print 'p est  = ', x_hat.item(2)*(180/np.pi)


# Main Function
def main():
    rospy.init_node('ekf_turtle', anonymous=True)
    r = rospy.Rate(10)
    pub1 = rospy.Publisher('/wp_ctrl/ekf', state_est , queue_size = 15)
    pub2 = rospy.Publisher('/ekf_compare', ekf_output, queue_size = 15)

    sub1 = rospy.Subscriber('/mobile_base/commands/velocity' , Twist        , callback1    )
    sub2 = rospy.Subscriber('/sensor_meas_rng'		         , sensor_op    , callback2    )
    sub3 = rospy.Subscriber('/mocap_raw/simple'		         , bot_states   , callback3    )
    # sub4 = rospy.Subscriber('/sensor_meas'		         , sensor_op    , callback4    )
    sub4 = rospy.Subscriber('/mobile_base/sensors/imu_data'  , Imu        , callback4    )
    sub5 = rospy.Subscriber('/ekf_turtle/param', param_turt_ekf             , callbackparam)

    print "EKF node running..."

    while not rospy.is_shutdown():

        if (start_flag == 1):
            ekf()

            est_states 	    	= state_est()
            est_states.x_est    = x_hat.item(0)
            est_states.y_est    = x_hat.item(1)
            est_states.psi_est  = x_hat.item(2)

            pub1.publish(est_states)	

            ekf_cmp		=  ekf_output()
            ekf_cmp.x_t 	=  x_true
            ekf_cmp.x_e 	=  x_hat.item(0)
            ekf_cmp.y_t 	=  y_true
            ekf_cmp.y_e 	=  x_hat.item(1)
            ekf_cmp.psi_t 	=  pi2pi(psi_true)
            ekf_cmp.psi_e 	=  x_hat.item(2)
            ekf_cmp.std_x	=  np.sqrt(P_hat[0,0])
            ekf_cmp.std_y	=  np.sqrt(P_hat[1,1])
            ekf_cmp.std_psi	=  np.sqrt(P_hat[2,2]) 

            pub2.publish(ekf_cmp)

        r.sleep()

    rospy.loginfo("EKF Node Has Shutdown...")
    rospy.signal_shutdown(0)


if __name__=='__main__':
    main()

