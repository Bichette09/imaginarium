#!/bin/bash
source ../devel/setup.sh
export ROS_MASTER_URI=http://192.168.1.35:11311
export ROS_IP=192.168.1.35
roslaunch gradingmachine_master.launch   
