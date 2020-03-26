#!/usr/bin/env python

# license removed for brevity

import sys						                # System tools
import math						                # Mathematical Functions
import rospy					                # Rospy is used for the Subscriber
import numpy 				           as np	# np for MATLAB like matrix operations
from time 		         import sleep  as t	    # Time to invoke pause time if required
from turtle_files.msg	 import *			    # Publish as Sensor Node
from dji_m100.msg import AnchorDist

# LM_x = np.array([0.0, -1.0,  0.0,  2.0])
# LM_y = np.array([0.0,  2.0, -3.0, -2.0])

LM_IdIn = -1
LM_DistIn = -100

################# Parameters to Integrate Sensor with EKF ################
num_LM      = 4 		 	 		### FIXME ##   # Hardcoded Value
meas_dist   = 3.0
rho_sensor  = np.array([-100.0])	### TODO ###   # Buffer to store range   measurements
sensor_flag =  1			 	 	# Signal  EKF  to  wait  till  Sensor is Transmitting
tag_id      = -1*np.ones(num_LM)
InputDataScale = 1000 #mm


#################### Function to Initiate  Subscriber #################### 
def callback_tag(data):
    global tag_id, rho_sensor, num_LM, InputDataScale

    rho_sensor = -100.0*np.ones(num_LM)

    try:
        LM_IdIn = data.anchor_ids
    	LM_DistIn = data.anchor_dists

    	for i in xrange(0,len(LM_IdIn)):

            if LM_IdIn[i] == '0235':
            	rho_sensor[0] = float(LM_DistIn[i])/InputDataScale
            	# print float(LM_DistIn[i])/InputDataScale

            elif LM_IdIn[i] == 'd3b5':
            	rho_sensor[1] = float(LM_DistIn[i])/InputDataScale
            	# print LM_DistIn[i]

            elif LM_IdIn[i] == '16b8':
            	rho_sensor[2] = float(LM_DistIn[i])/InputDataScale
            	# print LM_DistIn[i]

            elif LM_IdIn[i] == '54b5':
            	rho_sensor[3] = float(LM_DistIn[i])/InputDataScale
            	# print LM_DistIn[i]
    except:
        LM_IdIn = -1
        LM_DistIn = -100
            

def main():

    rospy.init_node('range_ar_tag', anonymous=True)
    r = rospy.Rate(2)

    pub = rospy.Publisher('/sensor_meas_rng', sensor_op, queue_size = 1)	### TODO ###
    
    rospy.Subscriber('/dwm1001/rf_distance'   , AnchorDist   , callback_tag)
    # rospy.Subscriber('/ekf_turtle/param' , param_turt_ekf , callbackparam)

    print 'DWM Range Sensor Node Running...'

    while not rospy.is_shutdown():
        # getRange()

        snsr             = sensor_op()					### TODO ###
        snsr.calc_range  = rho_sensor					### TODO ###
        snsr.flag_sensor = sensor_flag					### TODO ###
        pub.publish(snsr)

    r.sleep()

    rospy.loginfo("Shutting down in main")
    rospy.signal_shutdown(0)

if __name__ == '__main__':
    main()