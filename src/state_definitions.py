import random

#Here is where you can define all of your states in a clever way to make your code more readable.
#Think of all the states your robot could be in if thrown randomly into a world.
#Currently the code is set up for a variable name being equal to an integer like:

#FOLLOWING = 1
#for each of your states.

PI = 3.1415926
#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Random Wandering
def rand_wander(t):
    x_var = random.uniform(-0.1, 0.1)
    z_var = random.uniform(-0.1, 0.1)
    x_val = t.linear.x
    z_val = t.angular.z
    if x_val < 0.1 or x_val > 0.5:
        x_val = LINEAR_SPEED
    if z_val < -1 or z_val >1:
        z_val = ANGULAR_SPEED
    t.linear.x = x_val + x_var
    t.angular.z = z_val + z_var
    # t.linear.x = 0.3

#receives pid twist and publish to cmd_vel
def wall_follow(t, t_pid):
    t.linear.x = t_pid.linear.x 
    t.angular.z = t_pid.angular.z

#for rotating left
def rot_left(t, x):
    t.angular.z = ANGULAR_SPEED * x
    t.linear.x = 0.05

#for rotating right
def rot_right(t, x):
    t.angular.z = -1 * ANGULAR_SPEED * x
    t.linear.x = 0.05

