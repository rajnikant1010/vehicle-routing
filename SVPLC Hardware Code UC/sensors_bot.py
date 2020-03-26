#!/usr/bin/env python

# license removed for brevity

import rospy
import rospkg
import random
import numpy 		 	as np
import scipy.linalg 	as la

from math 		 			import *
from std_msgs.msg 	 		import String
from turtle_files.msg 	 	import sensor_op
from turtle_files.msg 	 	import param_turt_ekf
from waypoint_manager.msg   import bot_states
# from geometry_msgs.msg 	 import PoseStamped 

############# Landmark Parameters Initialization #############

LM_x       =  np.array([0.0, 0.0, 0.0])					# LM X-axis values
LM_y       =  np.array([0.0, 0.0, 0.0])					# LM Y-axis values

x_true	   = -1.0
y_true	   = -3.0
psi_true   = 0.0


num_LM     = -1
rho_sensor = np.array([-100.0])			# Buffer to store range measurements
br_sensor  = np.array([-100.0])	 		# Buffer to store bearing measurements
meas_dist  = 3.0				        # Maximum sensor range between LM and Vehicle
br_FOV_by2 = 45*(np.pi/180)

############ Desired Range Sensor Noise Level Std ############

rho_sensor_noise_power	= 0.01	 ### TODO ###   # Range   Sensor Noise Power Level
br_sensor_noise_power	= 0.01	 ### TODO ###   # Bearing Sensor Noise Power Level
sensor_flag		= 1			# Signal EKF to wait till Sensor is Transmitting

########################## -------- ##########################


# Function for Angle Wrapping
def pi2pi(angle):
    while((fabs(angle)- np.pi) >0.001):
        if (angle > np.pi):
            angle = angle - 2*np.pi
        if (angle <-np.pi):
            angle = angle + 2*np.pi
    return angle


# Callback to Get True States for Range Measurement
def callback1(data1):
    global x_true, y_true, psi_true

    x_true  	= data1.x
    y_true  	= data1.y
    psi_true 	= data1.psi*(np.pi/180)


# Callback to Get True States for Range Measurement
def callback2(data2):
    global LM_x, LM_y, num_LM, meas_dist
    global rho_sensor_noise_power, br_sensor_noise_power

    LM_x  					= np.asarray(data2.lm_x)
    LM_y  					= np.asarray(data2.lm_y)

    # original LMs 			- 0 if fake, 1 if original
    # original_LMs			= np.array([1,1,1,1,1,1,1,1,1,1])*(20000) # All Original
    # original_LMs			= np.array([0,0,0,0,0,0,0,0])         # All Fake
    LM_x 					= LM_x # + original_LMs
    LM_y 					= LM_y # + original_LMs

    num_LM					= data2.lm_num
    meas_dist				= data2.lm_meas_dist
    rho_sensor_noise_power	= data2.rho_sensor_noise_pow
    br_sensor_noise_power	= data2.br_sensor_noise_pow
    

# Obtain Range Measurement
def range_sensor():
    global LM_x, LM_y, num_LM, rho_sensor, meas_dist
    global x_true, y_true, rho_sensor_noise_power


    # if ((len(rho_sensor) != num_LM) and (num_LM > 0)): #FIXME: WRONG
    if (num_LM > 0):
        rho_sensor = -100.0*np.ones(num_LM)

    for i in xrange(0,num_LM):
        x_diff  = x_true - LM_x.item(i)
        y_diff  = y_true - LM_y.item(i)

        norm_sq = (np.power(x_diff, 2)) + (np.power(y_diff, 2))
        sqrt    = np.sqrt(norm_sq)

        if (sqrt <= meas_dist):
            rho_sensor[i] = sqrt + rho_sensor_noise_power*(random.random())

    # print rho_sensor

def range_sensor_FOV():						
    global LM_x, LM_y, num_LM, rho_sensor, meas_dist, br_FOV_by2
    global x_true, y_true, psi_true, rho_sensor_noise_power

    # Initialise array "br_sensor" with buffer value
    # if ((len(br_sensor) != num_LM) and (num_LM > 0)): #FIXME: WRONG
    if (num_LM > 0):
        rho_sensor = -100.0*np.ones(num_LM)

    for i in xrange(0,num_LM):

        x_diff  = x_true - LM_x.item(i)
        y_diff  = y_true - LM_y.item(i)

        norm_sq = (np.power(x_diff, 2)) + (np.power(y_diff, 2))
        sqrt    = np.sqrt(norm_sq)

        angl    = np.arctan2(-y_diff,-x_diff)
        br_angl = pi2pi(angl - psi_true)

        if ((sqrt <= meas_dist) and (br_angl >= -br_FOV_by2) and (br_angl <= br_FOV_by2)):
            rho_sensor[i] = sqrt + rho_sensor_noise_power*(random.random())

    # print rho_sensor

def bearing_sensor():						
    global LM_x, LM_y, num_LM, br_sensor, meas_dist, br_FOV_by2
    global x_true, y_true, psi_true, br_sensor_noise_power

    # Initialise array "br_sensor" with buffer value
    # if ((len(br_sensor) != num_LM) and (num_LM > 0)): #FIXME: WRONG
    if (num_LM > 0):
        br_sensor = -100.0*np.ones(num_LM)

    for i in xrange(0,num_LM):

        x_diff  = x_true - LM_x.item(i)
        y_diff  = y_true - LM_y.item(i)

        norm_sq = (np.power(x_diff, 2)) + (np.power(y_diff, 2))
        sqrt    = np.sqrt(norm_sq)

        angl    = np.arctan2(-y_diff,-x_diff)
        br_angl = pi2pi(angl - psi_true)

        if ((sqrt <= meas_dist) and (br_angl >= -br_FOV_by2) and (br_angl <= br_FOV_by2)):
        # if (sqrt <= meas_dist):
            br_sensor[i] = pi2pi(br_angl + br_sensor_noise_power*(random.random()))

    # print br_sensor


# The Main Function
def main():
    rospy.init_node('sensors_ekf', anonymous=True)
    r = rospy.Rate(10)
    # pub = rospy.Publisher('/sensor_meas', sensor_op, queue_size = 15)
    pub = rospy.Publisher('/sensor_meas_rng2', sensor_op, queue_size = 15)

    # sub1 = rospy.Subscriber('/turtle1/pose', Pose,  callback1)
    sub1 = rospy.Subscriber('/mocap_raw/simple', bot_states,  callback1)
    sub2 = rospy.Subscriber('/ekf_turtle/param', param_turt_ekf,  callback2)

    print 'Sensors Node Running...'

    while not rospy.is_shutdown():

        range_sensor_FOV()
        bearing_sensor()
        # if (len(rho_sensor) == num_LM):
        if (len(rho_sensor) > 1):
            # print rho_sensor

            snsr 				= sensor_op()		
            snsr.calc_range     = rho_sensor		
            # snsr.calc_bearing   = br_sensor		
            snsr.flag_sensor    = sensor_flag		
        
            pub.publish(snsr)					

        r.sleep()

    rospy.loginfo("Sensors Node Has Shutdown...")
    rospy.signal_shutdown(0)


if __name__=='__main__':
    main()

