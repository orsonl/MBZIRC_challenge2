#!/bin/bash

# Copy from local machine to husky
scp -r $HOME/catkin_ws/src/kuri_mbzirc_challenge_2/ administrator@192.168.1.11:/home/administrator/catkin_ws/src/kuri_mbzirc_challenge_2

# Catkin_make
ssh -t administrator@192.168.1.11 'cd catkin_ws; catkin_make;'

# Copy from husky to pr2-head and catkin_make
ssh -t administrator@192.168.1.11 "
  scp -r /home/administrator/catkin_ws/src/kuri_mbzirc_challenge_2/ pr2-head@192.168.1.19:/home/pr2-head/kinect_ws/src/kuri_mbzirc_challenge_2
  ssh -t pr2-head@192.168.1.19 \"
    cd kinect_ws;
    catkin_make;
    \"
  "
