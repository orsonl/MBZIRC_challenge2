# MBZIRC Challenge 2 Detecting Panel

This package moves towards the panel and positions the robot in front of it while allowing enough space for the manipulation task.

## Running

Follow the steps below to run the package on the Husky

```
==Terminal 1==
ssh administrator@192.168.1.11
roslaunch husky_navigation move_base_mapless_demo.launch

==Terminal 2==
ssh pr2-head@192.168.1.19
roslaunch kinect2_bridge kinect2_bridge.launch

==Terminal 3==
ssh pr2-head@192.168.1.19
roslaunch kuri_mbzirc_challenge_2_detecting_panel ar_pose_single_kinect_husky.launch

==Terminal 4==
ssh pr2-head@192.168.1.19
roslaunch kuri_mbzirc_challenge_2_detecting_panel ar_pose_single_kinect_husky.launch
```
