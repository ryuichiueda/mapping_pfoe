#!/bin/bash

### do SLAM ###
roslaunch raspimouse_cartographer slam.launch

### choose the max size trajectory file ###
# please install gawk with `sudo apt install gawk`
ls -l /tmp/trajectory_*.tsv |
sort -k5,5n                 |
tail -n 1                   |
gawk '{a=$NF;gsub(/[_0-9]*/,"",a);print $NF,a}'      |
xargs cp

### do replay ###
roslaunch raspimouse_map_based_teach_and_replay replay.launch

