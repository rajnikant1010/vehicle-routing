#!/usr/bin/env python

# license removed for brevity

import sys
import rospy
import rospkg
import numpy 		        as np
import scipy.linalg 	    as la
import matplotlib.pyplot    as plt

from math 		            import *
from std_msgs.msg 	        import String
from geometry_msgs.msg 	    import Twist
from turtlesim.msg 	        import Pose
from turtle_files.msg 	    import state_est
from turtle_files.msg 	    import param_turt_ekf
from turtle_files.srv	    import *
from waypoint_manager.msg   import bot_states
# from geometry_msgs.msg   import PoseStamped 

n            = 0 					    # Number of Waypoints
xw           = np.array([0.0, 0.0])		# WP X-Axis Values
yw           = np.array([0.0, 0.0])		# WP Y-Axis Values
psidot       = 0					    # Initialize State Omega/Psidot

x_true       = 0					    # Initialize State X
y_true       = 0					    # Initialize State Y
psi_true     = 0					    # Initialize State Psi

x_estm       = 0					    # Initialize State X
y_estm       = 0					    # Initialize State Y
psi_estm     = 0					    # Initialize State Psi

wpcounter    = 0					    # Initialize Waypoint Counter
st_flag      = 0					    # Run with True (0) or Est State (1)
st_flag_prev = 0					    # Run with True (0) or Est State (1)
vel_in       = 0					    # Commanded Input Velocity
min_dist     = 0.0					    # Min. Distance to WP
wpcnt_old    = -1					    # WP Counter Previous

vel_flag     = 0 					    # Stopping condition

########## Function for Angle Wrapping ##########
def pi2pi(angle):
    while((fabs(angle)- np.pi) >0.001):
        if (angle > np.pi):
            angle = angle - 2*np.pi
        if (angle <-np.pi):
            angle = angle + 2*np.pi
    return angle


############ Callback for  Turtlebot ############
def callback(data1):
    global x_true, y_true, psi_true
    x_true	= data1.x
    y_true	= data1.y
    psi_true	= data1.psi*(np.pi/180)


########## Callback for Turtlesim True ##########
def callbacksim(data2):
    global x_true, y_true, psi_true
    x_true     = data2.x
    y_true     = data2.y
    psi_true   = data2.theta


########## Callback for Turtlesim  EKF ##########
def callbackekf(data3):
    global x_estm, y_estm, psi_estm
    x_estm     = data3.x_est
    y_estm     = data3.y_est
    psi_estm   = data3.psi_est


########## Callback for Turtlesim  EKF ##########
def callbackparam(data4):
    global min_dist, xw, yw, n, vel_in
    xw         = np.asarray(data4.wp_x)
    yw         = np.asarray(data4.wp_y)
    min_dist   = data4.wp_min_dist
    n	       = data4.wp_num
    vel_in     = data4.vel_in


##### ROS Service Call for  Control Switch  #####
def handle_toggle_auto(req):

    global st_flag

    if (st_flag == 0):
        st_flag = 1
        ret_str = 'Toggled to Estimated Value Control'
    elif(st_flag == 1):
        st_flag = 0
        ret_str = 'Toggled to True Value Control'

    print "Toggled to %s"%(st_flag)
    return ret_str


########## The WP Controller  Function ##########
def wp_controller(xn, yn, psi, prnt_str):
    global wpcounter, xw, yw, psidot

    psi_wrap   = pi2pi(psi)
    x_diff     = xw.item((wpcounter)) - xn
    y_diff     = yw.item((wpcounter)) - yn
    psid       = np.arctan2(y_diff,x_diff)
    kp         = 0.6 # 1.0
    psidot     = kp*pi2pi(psid-psi_wrap)

    print prnt_str
    return psidot


################# Main Function #################
def main():

    global wpcounter, wpcnt_old, st_flag_prev, st_flag, vel_flag #, vel_in

    rospy.init_node('control_multi', anonymous=True)
    r = rospy.Rate(10)
    # pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 15)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 15)

    st_flag   = rospy.get_param("~state_flag", "0")
    st_flag   = int(st_flag)

    toggle_service = rospy.Service('toggle_srv_ekf', Toggle_EKF_WP, handle_toggle_auto)

    sub1 = rospy.Subscriber('/mocap_raw/simple', bot_states , callback)	#FIXME####
    sub2 = rospy.Subscriber('/wp_ctrl/ekf'     , state_est     , callbackekf)
    sub3 = rospy.Subscriber('/ekf_turtle/param', param_turt_ekf, callbackparam)

    while not rospy.is_shutdown():

        if ((st_flag == 0) and (st_flag_prev != st_flag)):
            print 'Running with True States'
            st_flag_prev = st_flag

        if ((st_flag == 1) and (st_flag_prev != st_flag)):
            print 'Running with Estimated States'
            st_flag_prev = st_flag


        if (st_flag == 0):	    
            x_sw    = x_true
            y_sw    = y_true
            psi_sw  = psi_true
            pr_str  = 'true_st'	    

        if (st_flag == 1):
            x_sw    = x_estm
            y_sw    = y_estm
            psi_sw  = psi_estm
            pr_str  = 'est_st'	    

        p_dot = wp_controller(x_sw, y_sw, psi_sw, pr_str)

        cmd           = Twist()
        #cmd.linear.x = vel_in
        cmd.linear.y  = 0
        cmd.linear.z  = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = p_dot # psidot

        if (wpcounter != wpcnt_old):
            print 'WP Number = ', wpcounter
            wpcnt_old = wpcounter

        if ((fabs(xw.item(wpcounter)-x_sw)< min_dist) and (fabs(yw.item(wpcounter)-y_sw)< min_dist)):
            if (wpcounter <n):
                wpcounter = wpcounter+1

            if (wpcounter == n):
                # print 'Should Stop, n = ', n
                # wpcounter 	= 0
                vel_flag	= 1
                rospy.loginfo("Reached final WP - Shutting down.")
                rospy.signal_shutdown(0)
        if (vel_flag == 1):
            cmd.linear.x = 0
            rospy.loginfo("Reached final WP - Shutting down.")
            rospy.signal_shutdown(0)
        else:
            print p_dot
            cmd.linear.x  = vel_in

        # print "velocity =", cmd.linear.x
        pub.publish(cmd)
        r.sleep()

    rospy.loginfo("Controller Node Has Shutdown.")
    rospy.signal_shutdown(0)

if __name__ == '__main__':
    main()
    
