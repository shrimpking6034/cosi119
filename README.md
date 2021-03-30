## General info
This project is created for PA6. A robot follows along a given line infinetely. 

## To start
roslaunch linefollower linemission.launch model:=waffle

## Description
It spawns a robot. A pre-created wolrd is loaded in Gazebo where a yellow line is drawn into. The robot, using computer vision, detects line and follows it.
 
## Explanation
line_follower.py file reads robot's camera data. Then, it filters by color in order to find yellow line. After detecting the line, using PID control, the robot follows along the line

