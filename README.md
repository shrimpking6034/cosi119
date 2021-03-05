## General info
This project is created for PA4. It is a robot that follows a wall.

## To start
roslaunch wall_follow wall_follower.launch

## Description
 Basic approach is similar to roomba, but instead, it follows wall if found.
 It starts by wandering around, and if finds a wall, it starts to follow.
 By using LaserScan, we find in which areas has walls. With sectionized scan datas, we detect how corner is shaped to follow.
 It constantly checks with PID for consistency
 