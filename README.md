# Exercise 1 - Mapping
## Introduction
This program is to start the simulation environment, launch the SLAM node, play a pre-recorded bag to control the robot along a suitable path, and automatically save the generated map.  
## How to run the program  
1. Enter and execute the following command in the terminal.  
```
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch tong_thesis_1 my_navigation_launch.py
```
2. Find the generated map's .yaml configuration file in the share directory of this package at the path *tong_thesis_1/maps/new_map.yaml*, and change *free_thresh* to 0.1.  
```
free_thresh: 0.1
```
## How to configure the program
The .lua configuration file for Cartograher can be found in the share directory of this package at the path *tong_thesis_1/cartographer_config/new_turtlebot3_lds_2d.lua*.  
# Exercise 2 - Navigation  
## Introduction
This program is to run the robot navigation program based on the map file generated in Exercise 1.  
## How to run the program  
1. Enter and execute the following command in the terminal.  
```
source /usr/share/gazebo/setup.sh
ros2 launch tong_thesis_1 my_navigation_launch.py
```
2. Click the *2D Pose Estimate* button in the RViz menu to set the initial position of the robot.  
3. Click the *2D Nav Goal* button in the RViz menu to set the navigation goal.  
## How to configure the program
The .yaml configuration file for Nav2 can be found in the share directory of this package at the path *tong_thesis_1/params/my_params.yaml*.  
