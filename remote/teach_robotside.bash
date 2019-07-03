#!/bin/bash -xv

exec 2> /tmp/$(basename $0).log

source ~/.bashrc
roslaunch raspimouse_cartographer slam_remote_robot.launch
