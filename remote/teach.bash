#!/bin/bash -xv

trap 'ssh ubuntu@raspimouse killall roslaunch' 1 2 3 15 

roscore &

sleep 5

ssh ubuntu@raspimouse "bash -i /home/ubuntu/catkin_ws/src/mapping_pfoe/remote/teach_robotside.bash" &

roslaunch raspimouse_cartographer slam_remote_desktop.launch 
