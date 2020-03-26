#!/usr/bin/env python

# license removed for brevity

import rospy
import rospkg
import numpy 		 as np
import scipy.linalg 	 as la

from math 		 import *
# from std_msgs.msg 	 import String
from turtle_files.msg 	 import param_turt_ekf


start_flag      = 1 # Starts EKF
update_ekf_flag = 1 # 1 for Range-only, 2 for Bearing-only, 3 for Both

######### Initialization of Turtle State - Velocity  #########

V_in       =  0.05
vel_std    =  0.00
omega_std  =  0.00

########### Initialization of Waypoints Multi EKF  ###########

RunCase = 4

if RunCase == 1:
    # Case 1: 4 LM Zig-zag Actual LM
    # X_Init: (0, -2, 90)

    xw = np.array([ 0.0, -0.7, 0.0, -0.7, 0.0])
    yw = np.array([-1.4, -0.7, 0.0,  0.7, 1.4])


elif RunCase == 2:
    # Case 2: 9 LM Zig-zag (Ran on 24, Aug 2019)
    # X_Init: (0, -6, 90)

    xw = np.array([ 0,  0, -1, -1,  0,  0,  1,  1,  0,  0, -1, -1, 0, 0])
    yw = np.array([-6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1,  0, 0, 1])

elif RunCase == 3:
    # Case 3: TSP Case (24 Apr, 2019) Simulated LM
    # X_Init: (-2, 1, 0)

    xw = np.array([-1.66, -0.35, -0.27,  0.06,  1.45,  1.58,  0.97,  1.24,  0.32,  1.12,  0.18, -1.24, -0.82, -0.30, -1.66])
    yw = np.array([ 1.13,  1.14,  0.76,  0.32,  0.86,  0.93, -1.52, -1.91, -3.49, -4.55, -5.25, -2.82, -2.68, -0.78,  1.13])

elif RunCase == 4:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM Case
    # X_Init: (-1, -3, -10)

    xw = np.array([-1.0, 0.0, 1.0, 1.2, 1.0, 0.0, -1.0, -1.2, -1.0])
    yw = np.array([-3.0, -3.2, -3.0, -2.0, -1.0, -0.8, -1.0, -2.0, -3.0])

elif RunCase == 5:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM Case
    # X_Init: (-1, -2, 0)

    xw = np.array([-1.0,  1.0, 2.0, 0.0, -1.0])
    yw = np.array([-2.0, -3.0, 1.0, 1.0, -2.0])

n          = len(xw)		# Number of Waypoints
min_dist   = 0.15			# Threshold to Change WP


############# Landmark Parameters Initialization #############

if RunCase == 1:
    # Case 1: 4 LM Zig-Zag Actual LM
    # X_Init: (0, -2, 90)

    # LM_x = np.array([-1.7, -1.1, 1.0, 0.4]) # LM Y-axis values
    # LM_y = np.array([ 0.3,  1.1, 1.0, 1.8]) # LM X-axis values

    LM_x = np.array([-0.9, -1.4, 0.7, 0.2]) # LM Y-axis values
    LM_y = np.array([ 1.4,  0.9, 1.6, 2.1]) # LM X-axis values


elif RunCase == 2:
    # Case 2: 9 LM Zig-Zag (Ran on 24, Aug 2019) Simulated LM
    # X_Init: (0, -6, 90)

    LM_x = np.array([-1.6, -1.4,  0.4,  1.4,  1.4, -1.4, -1.4, -0.4,  0.6]) # LM Y-axis values
    LM_y = np.array([-4.8, -3.6, -3.6, -4.0, -1.8, -1.4, -0.2,  1.0,  1.4]) # LM X-axis values

elif RunCase == 3:
    # Case 3: TSP Case (24 Apr, 2019) Simulated LM
    # X_Init: (-2, 1, 0)

    LM_x = np.array([ 1.51,  0.86,  1.88,  1.33, -1.68, -1.07, -0.53,  0.80, -1.25,  1.81,  1.22, -0.09, -1.70,  1.33, -1.01, -1.76]) # LM Y-axis values
    LM_y = np.array([-2.69, -0.16,  1.34, -4.80,  1.79, -3.03, -5.16,  0.07, -0.68,  0.87, -2.07, -5.39, -3.01, -5.16, -1.23,  1.55]) # LM X-axis values

elif RunCase == 4:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM
    # X_Init: (-1, -3, -10)

    LM_x = np.array([1.8, 1.6, 1.4, 0.9, -1.8, -1.6, -1.4, -0.9])
    LM_y = np.array([-3.4, -2.9, -0.2, -0.4, -0.6, -1.1, -3.8, -3.6])

elif RunCase == 5:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM
    # X_Init: (-1, -3, -10)

    LM_x = np.array([0.0, -1.0,  0.0,  2.0])
    LM_y = np.array([0.0,  2.0, -3.0, -2.0])

num_LM     =  len(LM_x)
meas_dist  =  3.0	# Maximum sensor range between LM and Vehicle


############ Desired Range Sensor Noise Level Std ############

rho_sensor_noise_power = 0.05	# Range   Sensor Noise Power Level
br_sensor_noise_power  = 0.01	# Bearing Sensor Noise Power Level


##################### EKF Initialization #####################

if RunCase == 1:
    # Case 1: 4 LM Zig-Zag Silumated LM
    x_est_init   =  0.0 			 # Meters
    y_est_init   = -2.0 			 # Meters
    psi_est_init =  90*(np.pi/180) 	 # Radians

elif RunCase == 2:
    # Case 2: 9 LM Zig-Zag (Ran on 24, Aug 2019) Simulated LM
    x_est_init   =  0.0 			 # Meters
    y_est_init   = -6.0 			 # Meters
    psi_est_init =  90*(np.pi/180) # Radians

elif RunCase == 3:
    # Case 3: TSP Case (24 Apr, 2019) Simulated LM
    x_est_init   = -2.0 			 # Meters
    y_est_init   =  1.0 			 # Meters
    psi_est_init =  0*(np.pi/180)  # Radians

elif RunCase == 4:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM
    x_est_init   = -1.0 			 # Meters
    y_est_init   = -3.0 			 # Meters
    psi_est_init = -10*(np.pi/180) # Radians

elif RunCase == 5:
    # Case 4: TSP case for paper (8th June, 2019) Actual LM
    x_est_init   = -1.0 			 # Meters
    y_est_init   = -2.0 			 # Meters
    psi_est_init =  0*(np.pi/180) # Radians

num_iter     = 10 				# Prediction Iteration

########################## -------- ##########################



# The Main Function
def main():
    rospy.init_node('param_ekf', anonymous=True)
    r = rospy.Rate(10)
    pub = rospy.Publisher('/ekf_turtle/param', param_turt_ekf, queue_size = 15)
    print 'Param Node Running...'

    while not rospy.is_shutdown():

        param_ekf 						= param_turt_ekf()
        param_ekf.vel_in 				= V_in
        param_ekf.wp_x 					= xw
        param_ekf.wp_y 					= yw
        param_ekf.wp_num 				= n
        param_ekf.wp_min_dist 			= min_dist
        param_ekf.lm_x 					= LM_x
        param_ekf.lm_y 					= LM_y
        param_ekf.lm_num 				= num_LM
        param_ekf.lm_meas_dist 			= meas_dist
        param_ekf.rho_sensor_noise_pow  = rho_sensor_noise_power
        param_ekf.br_sensor_noise_pow 	= br_sensor_noise_power
        param_ekf.ekf_x_init 			= x_est_init
        param_ekf.ekf_y_init 			= y_est_init
        param_ekf.ekf_psi_init 			= psi_est_init
        param_ekf.ekf_num_iter 			= num_iter
        param_ekf.ekf_vel_std 			= vel_std
        param_ekf.ekf_omega_std 		= omega_std
        param_ekf.flag_start 			= start_flag
        param_ekf.flag_update_ekf		= update_ekf_flag
        
        pub.publish(param_ekf)	
        #rospy.spin()
        r.sleep()

    rospy.loginfo("Param Turt EKF Node Has Shutdown...")
    rospy.signal_shutdown(0)


if __name__=='__main__':
    main()

