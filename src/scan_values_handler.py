#!/usr/bin/env python

#This processes all of the scan values


import rospy
import numpy as np
from wall_follow.msg import LidarFilter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32

dis = 0.5
state = 0
directions = {
    'f': 0,
    'fr':0,
    'r':0,
    'br':0,
    'b':0,
    'bl':0,
    'l':0,
    'fl':0,
}

#Process all the data from the LIDAR
def cb(msg):
    global ranges_filtered, state
    #Determine state
    ranges = np.array(msg.ranges)
    ranges_filtered = tf_scan(ranges)
    det_state(ranges_filtered)    
    pub_state.publish(state)
    #CALCULATE AND PUBLISH ANYTHING ELSE FOR THE PID
    pub_pid.publish(msg_handle(ranges_filtered))

#reshapes the scan data into 8 different sections. 
def tf_scan(ranges):
    ranges = valid_val(ranges)
    ranges = np.roll(ranges,23)
    ranges = np.reshape(ranges, (8,45))
    return ranges

#removes trash data.
def valid_val(ranges):
    ranges_filtered = []
    for x in ranges:
        if x == 'inf' or x > 3.8:
            ranges_filtered.append(4)
        else:
            ranges_filtered.append(x)
    return ranges_filtered


#0 random wander, 1 follow wall, 2 innerright 3 innerleft 4 outright 5 outleft
def det_state(ranges):
    global state, directions
    min_ranges = np.min(ranges, axis = 1)
    directions={
        'f': min_ranges[0],
        'fl':min_ranges[1],
        'l':min_ranges[2],
        'bl':min_ranges[3],
        'b':min_ranges[4],
        'br':min_ranges[5],
        'r':min_ranges[6],
        'fr':min_ranges[7],
    }
    closest = min(min_ranges)
    closest_dir = find_closest(ranges)
    if state == 0: #when wandering
        if closest <= dis:
            state = 1
    elif state  == 1: #when following wall
        #when found an inner corner
        if directions['f'] <dis and directions['fl'] < dis and directions['fr'] < dis and (directions['l'] < dis or directions['r'] < dis):
            if directions['l'] <directions['r']:
                state = 2
            else:
                state =3
        #when found an outer corner
        elif ((directions['br'] < dis and directions['r'] > dis and directions['fr']> dis and directions['f']>dis and directions['fl']>dis and directions['l'] >dis and directions['bl'] > dis)):
            state = 4
        elif ((directions['bl'] < dis and directions['r'] > dis and directions['fr']> dis and directions['f']>dis and directions['fl']>dis and directions['l'] >dis and directions['br'] > dis)):
            state = 5
    #when already in inner or outer corners, check it came out from corner
    elif  2 <= state <= 5: 
        if 2 <= state <=3 and directions['f'] > (dis*2):
            state = 1
        elif (4<= state <=5) and (closest_dir == 1 or closest_dir ==7):
            state = 1
    # print(state)

#returns closest index
def find_closest(ranges):
    return np.unravel_index(np.argmin(ranges), np.shape(ranges))[0]

#writes msg to custom made message.
def msg_handle(ranges):
    msg = LidarFilter()
    msg.front = ranges[0]
    msg.frontleft = ranges[1]
    msg.left = ranges[2]
    msg.backleft = ranges[3]
    msg.back = ranges[4]
    msg.backright = ranges[5]
    msg.right = ranges[6]
    msg.frontright = ranges[7]
    msg.closest = find_closest(ranges)
    return msg
        
#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
#THINK OF WHAT INFO TO PUBLISH TO THE PID
pub_pid = rospy.Publisher('lidar_filtered', LidarFilter, queue_size = 1)

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 