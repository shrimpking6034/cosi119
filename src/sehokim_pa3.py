#!/usr/bin/env python

import rospy
import numpy as np
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# fill in scan callback
def scan_cb(msg):
    global state;
    ranges = np.array(msg.ranges)
    #when obstacle is in below 0.2 meters, halt all.
    if(min(ranges))< 0.2:
        print("obstacle detected")
        state = "H"

# it is not necessary to add more code here but it could be useful
def key_cb(msg):
    global state; global last_key_press_time
    state = msg.data.upper()
    last_key_press_time = rospy.Time.now()

# odom for printing current pose
def odom_cb(msg):
    global pose
    pose = msg.pose

# print the state of the robot
def print_state(xspeed, zspeed):
    print("---")
    print("STATE: " + state)

    # calculate time since last key stroke
    time_since = rospy.Time.now() - last_key_press_time
    print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))
    try:
        print("CURRENT POSITION: (" + str(pose.pose.position.x) + ", " + str(pose.pose.position.y) + ")")
    except NameError:
        return
    print("CURRENT SPEED(LINEAR, ANGULAR): (" + str(xspeed) + ", " + str(zspeed) + ")")
    

# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time
state = "H"
last_key_press_time = rospy.Time.now()

#dictionary for all supporting movements' linear and angular component
movement_vector = {"H":[0, 0],"L":[0, 1],"R":[0, -1],"F":[1, 0],"B":[-1, 0],"S":[1, 1],"Z":[1,1]}
LINEAR_SPEED = 0.2
ANGULAR_SPEED = math.pi/4
z_counter = 0
# set rate
rate = rospy.Rate(10)

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
# publish cmd_vel from here 
    t = Twist()
    angular_transform = 1
    linear_transform = 1
    time_diff = rospy.Time.now().secs - last_key_press_time.secs

    #if keyboard input is not available,
    if state not in movement_vector:
        state = "H"
        print("INVALID COMMAND")
    #for spiralling motion
    elif state == "S" :
        angular_transform -= 0.03 * time_diff
    #for zigzag motion
    elif state == "Z" :
        z_counter += 1
        #turning left
        if  0 < time_diff % 9 <=2:
            linear_transform = 0
            angular_transform = 1
        #turning right
        elif 4< time_diff % 9 <=6:
            linear_transform = 0
            angular_transform = -1
        #going straight
        else:
            angular_transform = 0


    velocity_vector = movement_vector[state]
    t.linear.x = LINEAR_SPEED * velocity_vector[0] * linear_transform
    t.angular.z = ANGULAR_SPEED * velocity_vector[1] * angular_transform

    # print out the current state and time since last key press
    print_state(LINEAR_SPEED * velocity_vector[0] * linear_transform, ANGULAR_SPEED * velocity_vector[1] * angular_transform)

    cmd_vel_pub.publish(t)

    # run at 10hz
    rate.sleep()