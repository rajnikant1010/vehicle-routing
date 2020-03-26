#!/usr/bin/env python

# license removed for brevity

import sys						                # System tools
import math						                # Mathematical Functions
import rospy					                # Rospy is used for the Subscriber
import numpy 				           as np	# np for MATLAB like matrix operations
from time 		         import sleep  as t	    # Time to invoke pause time if required
from turtle_files.msg	 import *			    # Publish as Sensor Node
from ar_track_alvar_msgs.msg import AlvarMarkers


################# Parameters to Integrate Sensor with EKF ################
num_LM      = 9 		 	 		### FIXME ##   # Hardcoded Value
meas_dist   = 3.0
rho_sensor  = np.array([-100.0])	### TODO ###   # Buffer to store range   measurements
sensor_flag =  1			 	 	# Signal  EKF  to  wait  till  Sensor is Transmitting
tag_id      = -1*np.ones(num_LM)
tag_x       = -1*np.ones(num_LM)
tag_y       = -1*np.ones(num_LM)
tag_z       = -1*np.ones(num_LM)
bundleCnt   = 4



#################### Function to Initiate  Subscriber #################### 
def callback_tag(data):
    global tag_id, tag_x, tag_y, tag_z, num_LM

    for i in xrange(0,num_LM):
        try:
            tag_id[i] = data.markers[i].id
            tag_x[i]  = data.markers[i].pose.pose.position.x
            tag_y[i]  = data.markers[i].pose.pose.position.y
            tag_z[i]  = data.markers[i].pose.pose.position.z
        except:
            tag_id[i] = -1
            tag_x[i]  = -1
            tag_y[i]  = -1
            tag_z[i]  = -1

    


# ########## Callback for Turtlesim  EKF ##########
# def callbackparam(data):
#     global num_LM
#     num_LM	 = data.lm_num


def getRange():
    global tag_id, tag_x, tag_y, tag_z, rho_sensor, num_LM, bundleCnt, meas_dist

    rho_sensor = -100.0*np.ones(num_LM)

    for i in xrange(0,num_LM):
        if(tag_id[i] >=0):
            dist = math.sqrt(pow(tag_x[i],2)+pow(tag_y[i],2)) #+pow(tag_z[i],2))
            if(dist <= meas_dist):
                lm_val = int(tag_id[i]/bundleCnt)
                if (lm_val <= num_LM):
                    rho_sensor[lm_val] = dist
                print lm_val, dist

    # return(rho_sensor)





############################ The Main Function ###########################
def main():

    # global xxx

    rospy.init_node('range_ar_tag', anonymous=True)
    r = rospy.Rate(100)

    pub = rospy.Publisher('/sensor_meas_rng', sensor_op, queue_size = 15)	### TODO ###
    
    rospy.Subscriber('/ar_pose_marker'   , AlvarMarkers   , callback_tag)
    # rospy.Subscriber('/ekf_turtle/param' , param_turt_ekf , callbackparam)

    print 'AR_Tag Range Sensor Node Running...'

    while not rospy.is_shutdown():
        getRange()

        snsr 		    = sensor_op()					### TODO ###
        snsr.calc_range  = rho_sensor					### TODO ###
        snsr.flag_sensor = sensor_flag					### TODO ###
        pub.publish(snsr)

    r.sleep()

    rospy.loginfo("Shutting down in main")
    rospy.signal_shutdown(0)

if __name__ == '__main__':
    main()
