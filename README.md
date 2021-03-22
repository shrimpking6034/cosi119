## General info
This project is created for PA3. It simulates program similar to one of the teleop programs but with some interesting new wrinkles.

## To start
roslaunch double_follow double_follow.launch

## Description
It spawns 3 robots. User controls first robot with keyboard using teleop. Second robot follows first robot, and third robot follows second robot.
 
## Explanation
All robots streams their tf2 data. "turtle_tf2_listener.py" file listens the corresponding target's robot's tf and publishes follower robot's cmd_vel.
