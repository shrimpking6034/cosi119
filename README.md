## General info
This project is created for PA3. It simulates program similar to one of the teleop programs but with some interesting new wrinkles.

## To start
1. roscore
2. roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
3. rosrun SehoKim_PA3 key_publisher.py
4. rosrun SehoKim_PA3 sehokim_pa3.py

## Description
 Basic approach is similar to PA2, but instead of patrolling around, it gets command via keyboard and follows the order. 
 Once a key command is received, it looks up corresponding function from dictionary, which has supporting movements' linear and angular component.
 Once robot approaches obstacle in 0.2 meters using scan data, it halts all motion.
 
## Reflection
Trying to implement most of commands was fairly straight forward. But spiraling motion and zigzag motion was a bit harder.
By tracing on paper for spiraling and zigzag algorithms, it came more clear.

## Video orders
l: rotate left
r: rotate right
f: move forward
b: move backward
h: halt all motion
s: spiraling motion (like a curl or a spring)
z: zigzag motion