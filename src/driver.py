#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

state = 0 #Some starting state
states = {
    0: "wandering", 1: "wall following", 2: "inner right",
    3: "inner left", 4: "outer right", 5: "outer left"
}

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

print("STARTING")
#0 random wander, 1 follow wall, 2 innerright 3 innerleft 4 outright 5 outleft
while not rospy.is_shutdown():
    # print("STATE: ", states[state])
    if (state == 0):
        rand_wander(t_pub)
    elif (state == 1):
        wall_follow(t_pub, t_pid)
    elif (state == 2 or state ==4):
        rot_right(t_pub, 1)
    elif (state == 3 or state ==5):
        rot_left(t_pub, 1)
    else:
        print("STATE NOT FOUND")
    pub_vel.publish(t_pub)
    rate.sleep()
