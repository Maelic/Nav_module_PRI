#!/bin/sh

gnome-terminal --command=" bash -c 'roscore; rosparam set use_sim_time true; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; $SHELL'"&
gnome-terminal --command=" bash -c 'roslaunch pepper_nav_bringup mapping.launch; $SHELL'"&

rviz rviz
