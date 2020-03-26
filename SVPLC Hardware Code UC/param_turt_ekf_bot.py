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

# xw = np.array([0.5,-0.5, 0.0] ) + 0.00  # WP X-Axis Values
# yw = np.array([1.0, 3.0, 4.5] ) + 0.00  # WP Y-Axis Values

# xw = np.array([-0.4,  2.0,  2.0, -0.4, -0.4] ) + 0.00  # WP X-Axis Values
# yw = np.array([-0.4, -0.4,  2.0,  2.0, -0.4] ) + 0.00  # WP Y-Axis Values


# # 10 LM 3 Cam setup
 
# xw = np.array([ 1.6,  1.6, -1.6, -1.6,  1.6] ) + 0.00  # WP X-Axis Values
# yw = np.array([-5.6, -2.4, -2.4, -5.6, -5.6] ) + 2.00  # WP Y-Axis Values


# # 8 LM 1 Cam setup

# xw = np.array([ 0.9,  0.9, -0.9, -0.9,  0.9] ) + 0.00  # WP X-Axis Values
# yw = np.array([-3.6, -1.8, -1.8, -3.6, -3.6] ) + 0.00  # WP Y-Axis Values


# # 9 LM Zig-zag (Ran on 24, Aug 2019)

# xw = np.array([ 0,  0, -1, -1,  0,  0,  1,  1,  0,  0, -1, -1, 0, 0])
# yw = np.array([-6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1,  0, 0, 1])


# # 8 LM Zig-zag 2 (05 Apr, 2019)

# xw = np.array([-0.7,  0.0, -0.7,  0.0, -0.7,  0,  -0.7])
# yw = np.array([-4.2, -3.5, -2.8, -2.1, -1.4, -0.7, 0.0])


# # 8 LM Zig-zag (05 Apr, 2019) Run this case

# xw = np.array([0.6, -0.6, 0.6, -0.6, 0.6])
# yw = np.array([-4.8, -3.6, -2.4, -1.2, 0])


# # 8 LM Zig-zag (08 Apr, 2019) Run this case

# xw = np.array([-1.2,  0.0,  1.2,  1.0,  1.2,  0.0, -1.2, -1.0, -1.2])
# yw = np.array([-3.2, -3.4, -3.2, -2.0, -0.8, -0.6, -0.8, -2.0, -3.2])


## 4 LM Zig-zag

#xw = np.array([ 0.0, -0.7, 0.0, -0.7, 0.0])
#yw = np.array([-1.4, -0.7, 0.0,  0.7, 1.4])

# TSP case for paper (8th June, 2019)

# xw = np.array([-1.0, 0.0, 1.0, 1.2, 1.0, 0.0, -1.0, -1.2, -1.0])
# yw = np.array([-3.0, -3.2, -3.0, -2.0, -1.0, -0.8, -1.0, -2.0, -3.0])


# TSP Case (24 Apr, 2019) Run this case (-2.05, 1.05, 0-deg)

xw = np.array([-1.66, -0.35, -0.27,  0.06,  1.45,  1.58,  0.97,  1.24,  0.32,  1.12,  0.18, -1.24, -0.82, -0.30, -1.66])
yw = np.array([ 1.13,  1.14,  0.76,  0.32,  0.86,  0.93, -1.52, -1.91, -3.49, -4.55, -5.25, -2.82, -2.68, -0.78,  1.13])


n          = len(xw)			# Number of Waypoints
min_dist   = 0.15			# Threshold to Change WP


############# Landmark Parameters Initialization #############

# LM_x = np.array([1.0,-1.0, 1.0,-1.0, 1.0,-1.0]) # LM Y-axis values
# LM_y = np.array([2.0, 2.0, 4.0, 4.0, 6.0, 6.0])	# LM X-axis values


# 10 LM 3 Cam setup

# LM_x = np.array([0.6, -0.6, 0.6, -0.6, 0.6, -0.6, 1.7, -1.7, 2.0, -2.0]) + 0.00 # LM Y-axis values
# LM_y = np.array([-6.0, -6.0, -4.0, -4.0, -2.0, -2.0, -6.0, -2.0, -4.6, -3.4]) + 2.00 # LM X-axis values


# 8 LM 1 Cam setup

# LM_x = np.array([1.95, 1.3, 0.8, 1.5, -1.5, -0.8, -1.3, -1.95]) # LM Y-axis values
# LM_y = np.array([-4.2, -3.5, -1.4, -0.75, -4.65, -4.0, -1.9, -1.2]) # LM X-axis values

# LM_x = np.array([-0.70,  0.28,  2.17, 2.30, 1.00, 2.35, -0.83, -0.8]) # LM Y-axis values
# LM_y = np.array([-0.75, -0.66, -1.54, 0.55, 2.47, 2.49,  2.92,  1.00]) # LM X-axis values


# # 9 LM Zig-Zag (Ran on 24, Aug 2019)

# LM_x = np.array([-1.6, -1.4,  0.4,  1.4,  1.4, -1.4, -1.4, -0.4,  0.6]) # LM Y-axis values
# LM_y = np.array([-4.8, -3.6, -3.6, -4.0, -1.8, -1.4, -0.2,  1.0,  1.4]) # LM X-axis values


# # 8 LM Zig-Zag 2 (05 Apr, 2019)

# LM_x = np.array([-1.1, -1.1, -1.1, -1.1,  0.4,  0.4,  0.4, 0.4]) # LM Y-axis values
# LM_y = np.array([-2.5, -1.2,  0.2,  1.4, -3.2, -1.9, -0.5, 0.7]) # LM X-axis values


# # 8 LM Zig-Zag (05 Apr, 2019) Run this case

# LM_x = np.array([-1.4, -1.4, -1.0, -1.0,  1.0, 1.0,  1.4, 1.4]) # LM Y-axis values
# LM_y = np.array([-2.8, -0.4, -3.5, -1.1, -2.3, 0.1, -1.6, 0.8]) # LM X-axis values


# # 8 LM Zig-Zag (08 Apr, 2019) Run this case

# LM_x = np.array([ 2.0,  1.6,  1.3,  0.8,  -2.0, -1.60, -1.3, -0.80]) # LM Y-axis values
# LM_y = np.array([-3.1, -2.75, 0.0, -0.35, -0.9, -1.25, -4.0, -3.65]) # LM X-axis values


# 4 LM Zig-Zag

#LM_x = np.array([-1.7, -1.1, 1.0, 0.4]) # LM Y-axis values
#LM_y = np.array([ 0.3,  1.1, 1.0, 1.8]) # LM X-axis values

# TSP case for paper (8th June, 2019)

# LM_x = np.array([1.8, 1.6, 1.4, 0.9, -1.8, -1.6, -1.4, -0.9])
# LM_y = np.array([-3.4, -2.9, -0.2, -0.4, -0.6, -1.1, -3.8, -3.6])

# TSP Case (24 Apr, 2019) Run this case

LM_x = np.array([ 1.51,  0.86,  1.88,  1.33, -1.68, -1.07, -0.53,  0.80, -1.25,  1.81,  1.22, -0.09, -1.70,  1.33, -1.01, -1.76]) # LM Y-axis values
LM_y = np.array([-2.69, -0.16,  1.34, -4.80,  1.79, -3.03, -5.16,  0.07, -0.68,  0.87, -2.07, -5.39, -3.01, -5.16, -1.23,  1.55]) # LM X-axis values


num_LM     =  len(LM_x)
meas_dist  =  3.0	# Maximum sensor range between LM and Vehicle


############ Desired Range Sensor Noise Level Std ############

rho_sensor_noise_power = 0.05	# Range   Sensor Noise Power Level
br_sensor_noise_power  = 0.01	# Bearing Sensor Noise Power Level


##################### EKF Initialization #####################

x_est_init   = -1.60			# Meters
y_est_init   =  1.1			# Meters
psi_est_init =  0*(np.pi/180)			# Radians -98*(np.pi/180)

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

