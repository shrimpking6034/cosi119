#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
import numpy as np
from state_definitions import *
from geometry_msgs.msg import Twist
from wall_follow.msg import LidarFilter
from std_msgs.msg import Int16, Float32

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6
PI = 3.1415926
DIR = 1

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 0.5
#Integral constant
I_CONSTANT = 0.0
#Derivative constant
D_CONSTANT = 0.8

#CALLBACKS FOR ANYTHING YOUR PID NEEDS TO SUBSCRIBE TO FROM scan_values_handler

#received filtered data published by scan_values_handler
def cb_filtscan(msg):
    global measured_value, min_wall_angle
    closest = msg.closest
    ranges = [msg.front, msg.frontright, msg.right, msg.backright, msg.back, msg.backleft, msg.left, msg.frontleft]
    measured_value = min(ranges[closest])
    if closest < 4:
        min_wall_angle = closest/ 4 * PI
    else:
        min_wall_angle = (2*PI - (closest/ 4 * PI))

# def cb_twist(msg):
#     global ANGULAR_SPEED, LINEAR_SPEED
#     ANGULAR_SPEED = msg.angular.z
#     LINEAR_SPEED = msg.linear.x

#
# def cb_state(msg):
#     global state_num
#     state_num = msg.data

#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('pid_twist', Twist, queue_size = 1)

#SUBSCRIBERS FOR THINGS FROM scan_values_handler YOU MIGHT WANT
sub_state = rospy.Subscriber('lidar_filtered', LidarFilter, cb_filtscan)
# sub_twist = rospy.Subscriber('cmd_vel', Twist, cb_twist)
# sub_state = rospy.Subscriber('state', Int16, cb_state)

#Twist and rate object
t = Twist()
rate = rospy.Rate(10)

min_wall_angle = 0
desired = 0.5
measured_value = 1
prev_error = 0
integral = 0
state_num = 0
while not rospy.is_shutdown():
    # if state_num == 0:
    #     t.linear.x = LINEAR_SPEED
    #     t.angular.z = 0
    #     pub.publish(t)
    #     continue
    # if state_num !=1:
    #     continue
 
    error = measured_value - desired #equation 1
    # print("Error" + str(error))
    derivative = error - prev_error
    #integral += error
    # print("PID running")
    angle = PI/2 - min_wall_angle #equation 3
    # print(angle*180 / PI)
    
    #calculate p component PD component = equation 2
    p_component = P_CONSTANT * error
    #calculate d component
    d_component = D_CONSTANT * derivative 
    # print("P" + str(p_component))
    # print("D" + str(d_component))
    # print("derivative" + str(derivative))

    PD_val = p_component + d_component
    P_val = P_CONSTANT * angle #equation 4
    PID_val = PD_val + P_val
    # print("PID" + str(PID_val))
    #calculate i component
    #i_component = I_CONSTANT * integral
    
    prev_error = error
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = PID_val
    t.linear.x = 0.1

    if  -0.3 < PID_val <0.3:
        t.linear.x = 0.3
    # Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 